#ifndef __MY_SEND_TEST_H
#define __MY_SEND_TEST_H

#include "stdint.h"


#define SEND_ESP_HEAD 0xBB
#define SEND_ESP_ADDR 0x10
#define SEND_ESP_END 0xEE


#define RESCUE_FRAME_LEN 9
#define RESCUE_HEAD_1    0xAA
#define RESCUE_HEAD_2    0xBB
#define RESCUE_TAIL      0xFF


#define DEVICE_BROADCAST 0x10
#define DEVICE_FLIGHT    0x20
#define DEVICE_CAR       0x30
#define DEVICE_GROUND    0x40



void my_send_esp_4_test(int Data_A, int Data_B, int Data_C, int Data_D);
void my_send_esp_1_test(int Data_A);
void my_send_maixcam(uint8_t code_type);
void my_send_esp_qr_message(uint8_t device,uint8_t qr_type,int16_t qr_x,int16_t qr_y);


#endif
