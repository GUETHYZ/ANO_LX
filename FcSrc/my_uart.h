#ifndef __MT_UART_H
#define __MT_UART_H

#include "stdint.h"

#define VISION_CODE_RED_X 0xCA
#define VISION_CODE_OLD_B 0xCB
#define VISION_CODE_OLD_C 0xCC
#define VISION_CODE_LINE  0xA1

#define VISION_CTRL_RELEASE 0x50
#define VISION_CTRL_RED     0x60
#define VISION_CTRL_LINE    0x61

typedef struct
{
    uint8_t code;
    uint8_t ctrl;
    int16_t x_cm;
    int16_t y_cm;
    uint32_t last_update_ms; /* 逻辑时间戳；任务层主要用帧计数做超时保护 */
    uint8_t valid;
} Vision_Frame_t;

// extern int16_t OpenMV_data_0,OpenMV_data_1,OpenMV_data_2;
extern int16_t esp_data_1, esp_data_2;
extern int16_t OpenMV_data_0, OpenMV_data_1, OpenMV_data_2;
extern int my_task_flag; // 任务标志位，0表示空闲，1*表示有任务
extern uint8_t my_slam_flag;

/* MaixCam 原有兼容变量：只表示红色×目标控制，不再被黑线帧覆盖。 */
extern volatile uint16_t g_maixcam_valid_frame_cnt;
extern volatile uint16_t g_maixcam_line_frame_cnt;
extern volatile uint16_t g_maixcam_red_frame_cnt;
extern volatile uint16_t g_maixcam_ctrl_frame_cnt;
extern volatile uint8_t g_maixcam_last_code;
extern volatile uint8_t g_maixcam_last_ctrl;
extern volatile int16_t g_maixcam_last_x;
extern volatile int16_t g_maixcam_last_y;

extern volatile Vision_Frame_t latest_line_frame;
extern volatile Vision_Frame_t latest_red_frame;
extern volatile Vision_Frame_t latest_ctrl_frame;

void MY_uart_esp_receive(uint8_t data);
int MY_uart_esp_anl(uint8_t *data, uint8_t len);

void MY_uart_K230_receive(uint8_t data);
void MY_uart_K230_anl(uint8_t data);
void MY_uart_K230_send(uint8_t data);

void MY_uart_radar_receive(uint8_t data);
int MY_uart_radar_anl(uint8_t *data, uint8_t len);
void MY_uart_radar_send(uint8_t data);

void MY_uart_maixcam_clear_state(void);
void MY_uart_maixcam_send(uint8_t code_type);
void MY_uart_maixcam_receive(uint8_t data);
int MY_uart_maixcam_anl(uint8_t *data, uint8_t len);

#endif
