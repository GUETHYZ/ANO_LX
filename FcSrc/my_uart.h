#ifndef __MT_UART_H
#define __MT_UART_H

#include "stdint.h"

// extern int16_t OpenMV_data_0,OpenMV_data_1,OpenMV_data_2;
extern int16_t esp_data_1, esp_data_2;
extern int16_t OpenMV_data_0, OpenMV_data_1, OpenMV_data_2;
extern int my_task_flag; // 任务标志位，0表示空闲，1*表示有任务
extern uint8_t my_slam_flag;

extern volatile uint8_t g_radar3d_last_msg_id;
extern volatile int16_t g_radar3d_last_x;
extern volatile int16_t g_radar3d_last_y;
extern volatile int16_t g_radar3d_last_z;
extern volatile int16_t g_radar3d_last_roll;
extern volatile int16_t g_radar3d_last_pitch;
extern volatile int16_t g_radar3d_last_yaw;

extern volatile uint16_t g_maixcam_valid_frame_cnt;
extern volatile uint8_t g_maixcam_last_code;
extern volatile uint8_t g_maixcam_last_ctrl;
extern volatile int16_t g_maixcam_last_x;
extern volatile int16_t g_maixcam_last_y;

void MY_uart_esp_receive(uint8_t data);
int MY_uart_esp_anl(uint8_t *data, uint8_t len);

void MY_uart_K230_receive(uint8_t data);
void MY_uart_K230_anl(uint8_t data);
void MY_uart_K230_send(uint8_t data);

void MY_uart_radio_receive(uint8_t data);
int MY_uart_radio_anl(uint8_t *data, uint8_t len);
void MY_uart_radio_send(uint8_t data);

void MY_uart_maixcam_clear_state(void);
void MY_uart_maixcam_send(uint8_t code_type);
void MY_uart_maixcam_receive(uint8_t data);
int MY_uart_maixcam_anl(uint8_t *data, uint8_t len);

#endif
