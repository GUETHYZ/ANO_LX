#ifndef __MY_SEND_TEST_H
#define __MY_SEND_TEST_H

#include "stdint.h"


#define SEND_ESP_HEAD 0xBB
#define SEND_ESP_ADDR 0x10
#define SEND_ESP_END 0xEE

void my_send_esp_4_test(int Data_A, int Data_B, int Data_C, int Data_D);
void my_send_esp_1_test(int Data_A);
void my_send_maixcam(uint8_t code_type);


#endif
