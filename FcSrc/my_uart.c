#include "my_uart.h"
#include "Drv_Uart.h"
#include "my_get_data.h"
#include "my_send_test.h"
#include "my_contrl.h"

#define QR_CODE_A                 0xCA
#define QR_CODE_B                 0xCB
#define QR_CODE_C                 0xCC

#define ESP_FRAME_HEAD            0xBB
#define ESP_FRAME_ADDR            0x10
#define ESP_FRAME_FIXED           0xF1
#define ESP_FRAME_END             0xEE

#define ESP_FUNC_MISSION_CMD      0x10
#define ESP_FUNC_QR_TYPE          0x20

#define CAM_FRAME_HEAD            0xBB
#define CAM_FRAME_END             0xFF
#define CAM_CTRL_FC               0x50
#define CAM_CTRL_CAM              0x60

#define RADAR3D_FRAME_HEAD        0xAA
#define RADAR3D_FRAME_END         0x0A
#define RADAR3D_FRAME_LEN         15
#define RADAR3D_MSG_ID_MIN        0x01
#define RADAR3D_MSG_ID_MAX        0x03

int my_task_flag;                 /* 任务标志位，0表示空闲，1表示有任务 */
uint8_t my_slam_flag = 0;
int16_t OpenMV_data_0, OpenMV_data_1, OpenMV_data_2;

/* 三维激光雷达原始数据缓存：单位与雷达协议一致，位置为cm，姿态角为度。 */
volatile uint8_t g_radar3d_last_msg_id = 0;
volatile int16_t g_radar3d_last_x = 0;
volatile int16_t g_radar3d_last_y = 0;
volatile int16_t g_radar3d_last_z = 0;
volatile int16_t g_radar3d_last_roll = 0;
volatile int16_t g_radar3d_last_pitch = 0;
volatile int16_t g_radar3d_last_yaw = 0;

volatile uint16_t g_maixcam_valid_frame_cnt = 0;
volatile uint8_t g_maixcam_last_code = 0;
volatile uint8_t g_maixcam_last_ctrl = CAM_CTRL_FC;
volatile int16_t g_maixcam_last_x = 0;
volatile int16_t g_maixcam_last_y = 0;

static uint8_t is_valid_qr_type(uint8_t code)
{
    return (code == QR_CODE_A || code == QR_CODE_B || code == QR_CODE_C) ? 1u : 0u;
}

static uint8_t is_valid_cam_control(uint8_t control)
{
    return (control == CAM_CTRL_FC || control == CAM_CTRL_CAM) ? 1u : 0u;
}

void MY_uart_maixcam_clear_state(void)
{
    cam_target_control_flag = CAM_CTRL_FC;
    dis_x_cam_target = 0;
    dis_y_cam_target = 0;

    g_maixcam_last_ctrl = CAM_CTRL_FC;
    g_maixcam_last_x = 0;
    g_maixcam_last_y = 0;
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
            my_send_esp_1_test(0xDF);     /* 校验失败 */
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
 * func=0x20：地面站选择二维码类型。这里采用“立即转发 + 本地存储”的方案：
 *            1) cam_target_code_identity_flag 保存当前目标二维码；
 *            2) 立即通过 UART3 发送 BB code FF 给 MaixCam；
 *            3) 飞控任务函数后续只读取 cam_target_code_identity_flag，不再重复解析ESP帧。
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
        case 0x00: return 0;
        case 0x01: return 1;
        case 0x02: return 2;
        case 0x03: return 3;
        case 0x04: return 4;
        default:   return -1;
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
        // if(message==QR_CODE_A)
        // {
        //     my_send_esp_qr_message(DEVICE_BROADCAST, message, -111, 222);
        // }
        // else if (message==QR_CODE_B)
        // {
        //     my_send_esp_qr_message(DEVICE_BROADCAST, message, -333, 444);
        // }
        // else if (message==QR_CODE_C)
        // {
        //     my_send_esp_qr_message(DEVICE_BROADCAST, message, -555, 666);
        // }
        return 20;
    }

    return -1;
}

/* 三维激光雷达 -> 飞控：15字节帧
 * 波特率：115200，8N1
 * 格式：AA id xH xL yH yL zH zL rollH rollL pitchH pitchL yawH yawL 0A
 * 数据：X/Y/Z/roll/pitch/yaw 均为 int16，大端高字节在前。
 * 单位：X/Y/Z 为 cm，roll/pitch/yaw 为 °。
 * 示例：AA 01 00 64 FF CE 00 1E 00 0A FF FB 00 5A 0A
 *       x=100cm, y=-50cm, z=30cm, roll=10°, pitch=-5°, yaw=90°
 */
void MY_uart_radio_receive(uint8_t data)
{
    static uint8_t rxstate = 0;
    static uint8_t slam_datatemp[RADAR3D_FRAME_LEN];
    static uint8_t data_index = 0;

    /* 重同步：中途再次遇到帧头0xAA，认为新帧重新开始，避免丢字节后持续错位。 */
    if (data == RADAR3D_FRAME_HEAD && rxstate != 0)
    {
        data_index = 0;
        slam_datatemp[data_index++] = data;
        rxstate = 1;
        return;
    }

    switch (rxstate)
    {
    case 0:
        if (data == RADAR3D_FRAME_HEAD)
        {
            data_index = 0;
            slam_datatemp[data_index++] = data;
            rxstate = 1;
        }
        break;

    case 1:
        /* 消息ID：协议给出0x01/0x02/0x03，当前定位版通常只有0x01。 */
        if (data >= RADAR3D_MSG_ID_MIN && data <= RADAR3D_MSG_ID_MAX)
        {
            slam_datatemp[data_index++] = data;
            rxstate = 2;
        }
        else
        {
            data_index = 0;
            rxstate = 0;
        }
        break;

    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
        slam_datatemp[data_index++] = data;
        rxstate++;
        break;

    case 14:
        slam_datatemp[data_index++] = data;
        if (data_index == RADAR3D_FRAME_LEN &&
            slam_datatemp[0] == RADAR3D_FRAME_HEAD &&
            slam_datatemp[14] == RADAR3D_FRAME_END)
        {
            if (MY_uart_radio_anl(slam_datatemp, RADAR3D_FRAME_LEN) == 0)
            {
                my_slam_flag = 1;
            }
            else
            {
                my_slam_flag = 0;
                my_send_esp_1_test(0xDE);
            }
        }
        else
        {
            my_slam_flag = 0;
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

static int16_t radar3d_get_int16_be(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

int MY_uart_radio_anl(uint8_t *data, uint8_t len)
{
    int16_t x_position;
    int16_t y_position;
    int16_t z_position;
    int16_t roll_angle;
    int16_t pitch_angle;
    int16_t yaw_angle;

    if (len != RADAR3D_FRAME_LEN)
    {
        return -1;
    }

    if (data[0] != RADAR3D_FRAME_HEAD || data[14] != RADAR3D_FRAME_END)
    {
        return -1;
    }

    if (data[1] < RADAR3D_MSG_ID_MIN || data[1] > RADAR3D_MSG_ID_MAX)
    {
        return -1;
    }

    x_position  = radar3d_get_int16_be(data[2],  data[3]);
    y_position  = radar3d_get_int16_be(data[4],  data[5]);
    z_position  = radar3d_get_int16_be(data[6],  data[7]);
    roll_angle  = radar3d_get_int16_be(data[8],  data[9]);
    pitch_angle = radar3d_get_int16_be(data[10], data[11]);
    yaw_angle   = radar3d_get_int16_be(data[12], data[13]);

    g_radar3d_last_msg_id = data[1];
    g_radar3d_last_x = x_position;
    g_radar3d_last_y = y_position;
    g_radar3d_last_z = z_position;
    g_radar3d_last_roll = roll_angle;
    g_radar3d_last_pitch = pitch_angle;
    g_radar3d_last_yaw = yaw_angle;

    /*
     * 保持原二维雷达代码中的坐标约定：
     * 原代码将雷达x/y取反后写入 dis_x_slam/dis_y_slam，因此这里继续取反，
     * 避免后续路径点、PID方向和set_dis_zero()逻辑整体反向。
     * 新协议中的yaw已经是“度”，不再乘0.1。
     */
    dis_x_slam = -x_position;
    dis_y_slam = -y_position;
    yaw_slam = (float)yaw_angle;

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

/* MaixCam -> 飞控：BB code control signX xH xL signY yH yL FF */
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

    if (!is_valid_qr_type(code_type) || !is_valid_cam_control(control_bit))
    {
        return 0;
    }

    if ((data[3] != 0x00 && data[3] != 0x01) ||
        (data[6] != 0x00 && data[6] != 0x01))
    {
        return 0;
    }

    /* MaixCam理论上只会返回当前指定二维码。若返回码与地面站指定码不一致，忽略该帧，避免误切视觉控制。 */
    if (is_valid_qr_type(cam_target_code_identity_flag) && code_type != cam_target_code_identity_flag)
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

    cam_target_control_flag = control_bit;
    g_maixcam_last_code = code_type;
    g_maixcam_last_ctrl = control_bit;
    g_maixcam_last_x = x_position;
    g_maixcam_last_y = y_position;

    if (control_bit == CAM_CTRL_CAM)
    {
        dis_x_cam_target = x_position;
        dis_y_cam_target = y_position;
    }
    else
    {
        /* 0x50：相机释放控制权。 */
        dis_x_cam_target = 0;
        dis_y_cam_target = 0;
    }

    /* 只要是一帧合法MaixCam数据，就递增计数，供任务层判断数据是否持续刷新。 */
    g_maixcam_valid_frame_cnt++;

    return 1;
}
