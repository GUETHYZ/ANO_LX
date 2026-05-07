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

void my_send_maixcam(uint8_t code_type)
{
    uint8_t _cnt = 0;
    DataSend[_cnt++] = 0xBB;      // 帧头
    DataSend[_cnt++] = code_type; // 功能码
    DataSend[_cnt++] = 0xFF;      // 帧结束标志
    DrvUart3SendBuf(DataSend, 3);
}
