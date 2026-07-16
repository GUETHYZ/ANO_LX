#include "my_send_test.h"
#include "Drv_Uart.h"

uint8_t DataSend[30];

void my_send_esp_4_test(int Data_A, int Data_B, int Data_C, int Data_D)
{
    uint8_t _cnt = 0;
    DataSend[_cnt++] = SEND_ESP_HEAD; // 帧头
    DataSend[_cnt++] = SEND_ESP_ADDR; // ESP目标地址
    DataSend[_cnt++] = 0xF1;          // 功能码
    DataSend[_cnt++] = 0x08;          // 数据长度
    DataSend[_cnt++] = BYTE0(Data_A); // 填充数据
    DataSend[_cnt++] = BYTE1(Data_A); // 填充数据
    DataSend[_cnt++] = BYTE0(Data_B); // 填充数据
    DataSend[_cnt++] = BYTE1(Data_B); // 填充数据
    DataSend[_cnt++] = BYTE0(Data_C); // 填充数据
    DataSend[_cnt++] = BYTE1(Data_C); // 填充数据
    DataSend[_cnt++] = BYTE0(Data_D); // 填充数据
    DataSend[_cnt++] = BYTE1(Data_D); // 填充数据
    DataSend[_cnt++] = SEND_ESP_END;  // 帧结束标志

    uint8_t sc = 0;
    uint8_t ac = 0;
    for (uint8_t i = 0; i < DataSend[3] + 5; i++)
    {
        sc += DataSend[i];
        ac += sc;
    }
    DataSend[_cnt++] = sc; // 和校验位
    DataSend[_cnt++] = ac; // 附加 校验位

    DrvUart1SendBuf(DataSend, DataSend[3] + 7);
}

void my_send_esp_1_test(int Data_A)
{
    uint8_t _cnt = 0;
    DataSend[_cnt++] = SEND_ESP_HEAD; // 帧头
    DataSend[_cnt++] = SEND_ESP_ADDR; // esp目标地址
    DataSend[_cnt++] = 0xF1;          // 功能码
    DataSend[_cnt++] = 0x02;          // 数据长度
    DataSend[_cnt++] = BYTE0(Data_A); // 填充数据
    DataSend[_cnt++] = BYTE1(Data_A); // 填充数据
    DataSend[_cnt++] = SEND_ESP_END;  // 帧结束标志

    uint8_t sc = 0;
    uint8_t ac = 0;
    for (uint8_t i = 0; i < DataSend[3] + 5; i++)
    {
        sc += DataSend[i];
        ac += sc;
    }
    DataSend[_cnt++] = sc; // 和校验位
    DataSend[_cnt++] = ac; // 附加 校验位

    DrvUart1SendBuf(DataSend, DataSend[3] + 7);
}

void my_send_esp_qr_message(uint8_t device, uint8_t qr_type, int16_t qr_x, int16_t qr_y)
{
    uint8_t _cnt = 0;
    uint16_t x_raw = (uint16_t)qr_x;
    uint16_t y_raw = (uint16_t)qr_y;

    /*
     * 通用通信协议：AA BB device qr_type xH xL yH yL FF
     * x/y 为 int16_t 补码，大端序发送，高字节在前。
     * 这样 ESP32 端可用 (frame[4] << 8) | frame[5] 还原为 int16_t。
     */
    DataSend[_cnt++] = RESCUE_HEAD_1;                  // 0xAA
    DataSend[_cnt++] = RESCUE_HEAD_2;                  // 0xBB
    DataSend[_cnt++] = device;                         // 目标设备 / 设备类型
    DataSend[_cnt++] = qr_type;                        // 二维码类型：0xCA / 0xCB / 0xCC
    DataSend[_cnt++] = (uint8_t)((x_raw >> 8) & 0xFF); // X 高
    DataSend[_cnt++] = (uint8_t)(x_raw & 0xFF);        // X 低字节
    DataSend[_cnt++] = (uint8_t)((y_raw >> 8) & 0xFF); // Y 高字节
    DataSend[_cnt++] = (uint8_t)(y_raw & 0xFF);        // Y 低字节
    DataSend[_cnt++] = RESCUE_TAIL;                    // 0xFF

    DrvUart1SendBuf(DataSend, RESCUE_FRAME_LEN);
}

void my_send_maixcam(uint8_t code_type)
{
    uint8_t _cnt = 0;
    DataSend[_cnt++] = 0xBB;      // 帧头
    DataSend[_cnt++] = code_type; // 功能码
    DataSend[_cnt++] = 0xFF;      // 帧结束标志
    DrvUart3SendBuf(DataSend, 3);
}
