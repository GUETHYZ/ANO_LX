#include "my_uart.h"
#include "Drv_Uart.h"
#include "my_get_data.h"
#include "my_send_test.h"
#include "my_contrl.h"

#define QR_CODE_A VISION_CODE_RED_X
#define QR_CODE_B VISION_CODE_OLD_B
#define QR_CODE_C VISION_CODE_OLD_C
#define LINE_CODE VISION_CODE_LINE

#define ESP_FRAME_HEAD 0xBB
#define ESP_FRAME_ADDR 0x10
#define ESP_FRAME_FIXED 0xF1
#define ESP_FRAME_END 0xEE

#define ESP_FUNC_MISSION_CMD 0x10
#define ESP_FUNC_QR_TYPE 0x20

#define CAM_FRAME_HEAD 0xBB
#define CAM_FRAME_END 0xFF
#define CAM_CTRL_FC VISION_CTRL_RELEASE
#define CAM_CTRL_CAM VISION_CTRL_RED
#define CAM_CTRL_LINE VISION_CTRL_LINE

#define RADAR3D_FRAME_HEAD 0xAA
#define RADAR3D_FRAME_END 0x0A
#define RADAR3D_FRAME_LEN 15
#define RADAR3D_MSG_ID_MIN 0x01
#define RADAR3D_MSG_ID_MAX 0x03

int my_task_flag; /* 任务标志位，0表示空闲，1表示有任务 */
uint8_t my_slam_flag = 0;
int16_t OpenMV_data_0, OpenMV_data_1, OpenMV_data_2;

volatile uint16_t g_maixcam_valid_frame_cnt = 0;
volatile uint16_t g_maixcam_line_frame_cnt = 0;
volatile uint16_t g_maixcam_red_frame_cnt = 0;
volatile uint16_t g_maixcam_ctrl_frame_cnt = 0;
volatile uint8_t g_maixcam_last_code = 0;
volatile uint8_t g_maixcam_last_ctrl = CAM_CTRL_FC;
volatile int16_t g_maixcam_last_x = 0;
volatile int16_t g_maixcam_last_y = 0;

volatile Vision_Frame_t latest_line_frame = {0};
volatile Vision_Frame_t latest_red_frame = {0};
volatile Vision_Frame_t latest_ctrl_frame = {0};

static uint8_t is_valid_qr_type(uint8_t code)
{
    return (code == QR_CODE_A || code == QR_CODE_B || code == QR_CODE_C) ? 1u : 0u;
}

static uint8_t is_valid_cam_code(uint8_t code)
{
    return (is_valid_qr_type(code) || code == LINE_CODE) ? 1u : 0u;
}

static uint8_t is_valid_cam_control(uint8_t control)
{
    return (control == CAM_CTRL_FC || control == CAM_CTRL_CAM || control == CAM_CTRL_LINE) ? 1u : 0u;
}

static uint8_t is_valid_cam_code_ctrl_pair(uint8_t code, uint8_t ctrl)
{
    if (code == LINE_CODE)
    {
        return (ctrl == CAM_CTRL_LINE) ? 1u : 0u;
    }

    if (is_valid_qr_type(code))
    {
        return (ctrl == CAM_CTRL_CAM || ctrl == CAM_CTRL_FC) ? 1u : 0u;
    }

    return 0;
}

static void save_vision_frame(volatile Vision_Frame_t *dst,
                              uint8_t code,
                              uint8_t ctrl,
                              int16_t x_cm,
                              int16_t y_cm)
{
    dst->code = code;
    dst->ctrl = ctrl;
    dst->x_cm = x_cm;
    dst->y_cm = y_cm;
    dst->last_update_ms = (uint32_t)g_maixcam_valid_frame_cnt * 20u;
    dst->valid = 1;
}

void MY_uart_maixcam_clear_state(void)
{
    cam_target_control_flag = CAM_CTRL_FC;
    dis_x_cam_target = 0;
    dis_y_cam_target = 0;

    g_maixcam_last_ctrl = CAM_CTRL_FC;
    g_maixcam_last_code = 0;
    g_maixcam_last_x = 0;
    g_maixcam_last_y = 0;

    latest_line_frame.valid = 0;
    latest_red_frame.valid = 0;
    latest_ctrl_frame.valid = 0;
}

/*
 * 兼容 my_uart.h 中已有声明。
 * 实际发送函数 my_send_maixcam() 已在 my_send_test.c 中实现。
 */
void MY_uart_maixcam_send(uint8_t code_type)
{
    if (!is_valid_qr_type(code_type))
    {
        return;
    }
    my_send_maixcam(code_type);
}

/* ESP32 -> 飞控：接收完整一帧后调用解析函数
 * 当前帧格式：BB 10 F1 func cmd EE SC AC
 */
void MY_uart_esp_receive(uint8_t data)
{
    static uint8_t rxstate = 0;
    static uint8_t esp_datatemp[8];
    static uint8_t data_index = 0;

    switch (rxstate)
    {
    case 0: /* 等待帧头 0xBB */
        if (data == ESP_FRAME_HEAD)
        {
            data_index = 0;
            esp_datatemp[data_index++] = data;
            rxstate = 1;
        }
        break;

    case 1: /* 地址 0x10 */
        if (data == ESP_FRAME_ADDR)
        {
            esp_datatemp[data_index++] = data;
            rxstate = 2;
        }
        else
        {
            rxstate = 0;
            data_index = 0;
        }
        break;

    case 2: /* 固定字节 0xF1 */
        if (data == ESP_FRAME_FIXED)
        {
            esp_datatemp[data_index++] = data;
            rxstate = 3;
        }
        else
        {
            rxstate = 0;
            data_index = 0;
        }
        break;

    case 3: /* func */
        esp_datatemp[data_index++] = data;
        rxstate = 4;
        break;

    case 4: /* cmd/message */
        esp_datatemp[data_index++] = data;
        rxstate = 5;
        break;

    case 5: /* 终止位 0xEE */
        if (data == ESP_FRAME_END)
        {
            esp_datatemp[data_index++] = data;
            rxstate = 6;
        }
        else
        {
            rxstate = 0;
            data_index = 0;
        }
        break;

    case 6: /* SC */
        esp_datatemp[data_index++] = data;
        rxstate = 7;
        break;

    case 7: /* AC */
    {
        uint8_t sc = 0;
        uint8_t ac = 0;
        uint8_t i;
        int ret;

        esp_datatemp[data_index++] = data;

        for (i = 0; i < 6; i++)
        {
            sc += esp_datatemp[i];
            ac += sc;
        }

        if (sc == esp_datatemp[6] && ac == esp_datatemp[7])
        {
            ret = MY_uart_esp_anl(esp_datatemp, 8);

            if (ret >= 0 && ret < 5)
            {
                my_task_flag = ret;
            }
            else if (ret < 0)
            {
                my_send_esp_1_test(0xDD); /* 格式合法但业务字段不合法 */
            }
        }
        else
        {
            my_send_esp_1_test(0xDF); /* 校验失败 */
        }

        rxstate = 0;
        data_index = 0;
        break;
    }

    default:
        rxstate = 0;
        data_index = 0;
        break;
    }
}

/* ESP32 -> 飞控：解析完整8字节帧
 * func=0x10：任务命令
 * func=0x20：地面站选择目标类型。E题实际只使用红色× code=0xCA；0xCB/0xCC 保留兼容。
 */
int MY_uart_esp_anl(uint8_t *data, uint8_t len)
{
    uint8_t func;
    uint8_t message;

    if (len != 8)
    {
        return -1;
    }

    if (data[0] != ESP_FRAME_HEAD || data[1] != ESP_FRAME_ADDR ||
        data[2] != ESP_FRAME_FIXED || data[5] != ESP_FRAME_END)
    {
        return -1;
    }

    func = data[3];
    message = data[4];

    if (func == ESP_FUNC_MISSION_CMD)
    {
        switch (message)
        {
        case 0x00:
            return 0;
        case 0x01:
            return 1;
        case 0x02:
            return 2;
        case 0x03:
            return 3;
        case 0x04:
            return 4;
        default:
            return -1;
        }
    }
    else if (func == ESP_FUNC_QR_TYPE)
    {
        if (!is_valid_qr_type(message))
        {
            return -1;
        }

        cam_target_code_identity_flag = message;
        cam_target_control_flag = CAM_CTRL_FC;
        dis_x_cam_target = 0;
        dis_y_cam_target = 0;

        MY_uart_maixcam_send(message);
        return 20;
    }

    return -1;
}

/* ---------------- 雷达接收：新协议实现 ---------------- */
void MY_uart_radar_receive(uint8_t data)
{
    static u8 rxstate = 0;
    static u8 radar_buf[14]; // 14字节缓冲区
    static u8 data_index = 0;

    switch (rxstate)
    {
    case 0: // 等待帧头 0xAA
        if (data == 0xAA)
        {
            radar_buf[0] = data;
            data_index = 1;
            rxstate = 1;
        }
        break;

    case 1: // 接收剩余 13 个字节（索引 1~13）
        radar_buf[data_index++] = data;
        if (data_index >= 14)
        {
            // 检查帧尾
            if (radar_buf[0] == 0xAA && radar_buf[13] == 0x0A)
            {
                MY_uart_radar_anl(radar_buf, 14);
                my_slam_flag = 1;
            }
            else
            {
                my_slam_flag = 0;
                my_send_esp_1_test(0xDE); // 帧错误反馈
            }
            data_index = 0;
            rxstate = 0;
        }
        break;

    default:
        rxstate = 0;
        data_index = 0;
        break;
    }
}

int MY_uart_radar_anl(uint8_t *data, uint8_t len)
{
    if (len != 14 || data[0] != 0xAA || data[13] != 0x0A)
    {
        return -1;
    }

    // 按照大端序解析 int16（高字节在前，低字节在后）
    int16_t x = (int16_t)((data[1] << 8) | data[2]);
    int16_t y = (int16_t)((data[3] << 8) | data[4]);
    int16_t z = (int16_t)((data[5] << 8) | data[6]);
    // roll 和 pitch 可根据需要忽略，这里仅存储但不发送
    // int16_t roll  = (int16_t)((data[7] << 8) | data[8]);
    // int16_t pitch = (int16_t)((data[9] << 8) | data[10]);
    int16_t yaw = (int16_t)((data[11] << 8) | data[12]);

    (void)z;

    // 更新全局变量
    dis_x_slam = x;
    dis_y_slam = y;
    yaw_slam = yaw;

    return 0;
}

void MY_uart_radio_send(uint8_t data)
{
    uint8_t Buf[] = {1, 0x0A};
    (void)data;
    DrvUart2SendBuf(Buf, sizeof(Buf));
}

/* 旧K230/OpenMV接收逻辑，保留兼容 */
void MY_uart_K230_receive(uint8_t data)
{
    static int data_cnt = 0;
    static uint8_t data_1, data_2, data_3, data_4, data_5;

    if ((data_cnt == 0) && (data == 0x0A))
    {
        data_cnt = 1;
    }
    else if (data_cnt == 1)
    {
        data_1 = data;
        data_cnt = 2;
    }
    else if (data_cnt == 2)
    {
        data_2 = data;
        data_cnt = 3;
    }
    else if (data_cnt == 3)
    {
        data_3 = data;
        data_cnt = 4;
    }
    else if (data_cnt == 4)
    {
        data_4 = data;
        data_cnt = 5;
    }
    else if (data_cnt == 5)
    {
        data_5 = data;
        data_cnt = 0;
    }
    else
    {
        data_cnt = 0;
    }

    OpenMV_data_0 = data_1;
    if (OpenMV_data_0 == 1)
    {
        OpenMV_data_1 = data_3;
        OpenMV_data_1 <<= 8;
        OpenMV_data_1 |= data_2;
        OpenMV_data_2 = data_5;
        OpenMV_data_2 <<= 8;
        OpenMV_data_2 |= data_4;
        OpenMV_data_1 *= 0.75f;
        OpenMV_data_2 *= 0.75f;
    }
    else
    {
        OpenMV_data_1 = 0;
        OpenMV_data_2 = 0;
    }
}

void MY_uart_K230_send(uint8_t data)
{
    uint8_t Buf[] = {1, 0x0A};
    (void)data;
    DrvUart3SendBuf(Buf, sizeof(Buf));
}

/* MaixCam -> 飞控：BB code ctrl signX xH xL signY yH yL FF */
void MY_uart_maixcam_receive(uint8_t data)
{
    static uint8_t rxstate = 0;
    static uint8_t maixcam_datatemp[10];
    static uint8_t data_index = 0;

    /* 重同步：中途再次遇到0xBB，认为是新帧开始，避免漏字节后持续错位。 */
    if (data == CAM_FRAME_HEAD && rxstate != 0)
    {
        data_index = 0;
        maixcam_datatemp[data_index++] = data;
        rxstate = 1;
        return;
    }

    switch (rxstate)
    {
    case 0:
        if (data == CAM_FRAME_HEAD)
        {
            data_index = 0;
            maixcam_datatemp[data_index++] = data;
            rxstate = 1;
        }
        break;

    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
        maixcam_datatemp[data_index++] = data;
        rxstate++;
        break;

    case 9:
        maixcam_datatemp[data_index++] = data;
        if (data_index == 10 && maixcam_datatemp[0] == CAM_FRAME_HEAD && maixcam_datatemp[9] == CAM_FRAME_END)
        {
            MY_uart_maixcam_anl(maixcam_datatemp, 10);
        }
        else
        {
            my_send_esp_1_test(0xDE);
        }
        data_index = 0;
        rxstate = 0;
        break;

    default:
        rxstate = 0;
        data_index = 0;
        break;
    }
}

int MY_uart_maixcam_anl(uint8_t *data, uint8_t len)
{
    uint8_t code_type;
    uint8_t control_bit;
    int16_t x_position;
    int16_t y_position;

    if (len != 10)
    {
        return 0;
    }

    if (data[0] != CAM_FRAME_HEAD || data[9] != CAM_FRAME_END)
    {
        return 0;
    }

    code_type = data[1];
    control_bit = data[2];

    if (!is_valid_cam_code(code_type) || !is_valid_cam_control(control_bit))
    {
        return 0;
    }

    if (!is_valid_cam_code_ctrl_pair(code_type, control_bit))
    {
        return 0;
    }

    if ((data[3] != 0x00 && data[3] != 0x01) ||
        (data[6] != 0x00 && data[6] != 0x01))
    {
        return 0;
    }

    x_position = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    y_position = (int16_t)(((uint16_t)data[7] << 8) | data[8]);

    if (data[3] == 0x01)
    {
        x_position = -x_position;
    }
    if (data[6] == 0x01)
    {
        y_position = -y_position;
    }

    g_maixcam_last_code = code_type;
    g_maixcam_last_ctrl = control_bit;
    g_maixcam_last_x = x_position;
    g_maixcam_last_y = y_position;

    /*
     * 0xA1/0x61：黑线巡线辅助。只更新 latest_line_frame，不覆盖红色×控制变量，
     * 否则巡线帧会把投掷状态机误触发或误释放。
     */
    if (code_type == LINE_CODE && control_bit == CAM_CTRL_LINE)
    {
        save_vision_frame(&latest_line_frame, code_type, control_bit, x_position, y_position);
        g_maixcam_line_frame_cnt++;
        g_maixcam_valid_frame_cnt++;
        return 1;
    }

    /* 红色×/兼容旧 code 的 0x60：末端引导偏差。 */
    if (is_valid_qr_type(code_type) && control_bit == CAM_CTRL_CAM)
    {
        /* 如果地面站仍然选择 0xCB/0xCC，则兼容；E题默认目标是 0xCA。 */
        if (is_valid_qr_type(cam_target_code_identity_flag) && code_type != cam_target_code_identity_flag)
        {
            return 0;
        }

        save_vision_frame(&latest_red_frame, code_type, control_bit, x_position, y_position);
        cam_target_control_flag = CAM_CTRL_CAM;
        dis_x_cam_target = x_position;
        dis_y_cam_target = y_position;
        g_maixcam_red_frame_cnt++;
        g_maixcam_valid_frame_cnt++;
        return 1;
    }

    /* 红色×/兼容旧 code 的 0x50：只表示释放视觉控制；是否为投放完成由任务状态机判定。 */
    if (is_valid_qr_type(code_type) && control_bit == CAM_CTRL_FC)
    {
        if (is_valid_qr_type(cam_target_code_identity_flag) && code_type != cam_target_code_identity_flag)
        {
            return 0;
        }

        save_vision_frame(&latest_ctrl_frame, code_type, control_bit, x_position, y_position);
        cam_target_control_flag = CAM_CTRL_FC;
        dis_x_cam_target = 0;
        dis_y_cam_target = 0;
        g_maixcam_ctrl_frame_cnt++;
        g_maixcam_valid_frame_cnt++;
        return 1;
    }

    return 0;
}
