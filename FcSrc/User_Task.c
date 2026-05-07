#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Ano_Math.h"

#include "my_contrl.h"
#include "my_get_data.h"
#include "my_send_test.h"
#include "my_uart.h"
#include "my_pid.h"

// ============================================================
// 雷达主控 + MaixCam辅助修正目标点版 User_Task.c
//
// 核心思路：
// 1. 飞机起飞后始终使用 mode_select(4)：雷达定点 + 定高 + yaw保持。
// 2. MaixCam 不再直接进入 mode_select(3)，不直接控制速度环。
// 3. CH7 开启后，飞控向相机发送 BB CA FF，请求识别 A码。
// 4. 相机返回的 dis_x_cam_target / dis_y_cam_target 被视为：二维码相对飞机的机体系偏差。
// 5. 任务层将“当前雷达坐标 + 相机偏差”转换成新的雷达目标点 dis_x_target / dis_y_target。
// 6. 相机短暂丢帧时，继续保持最后一次有效雷达目标点；长时间无更新后，回到当前位置雷达悬停。
//
// 重要：本文件不依赖 g_maixcam_valid_frame_cnt，因此只替换 User_Task.c 也能编译。
//      由于没有帧计数，本文件用“相机控制位/坐标变化”判断新样本，避免旧坐标被每周期重复累加。
//      后续更稳的做法是在 my_uart.c 中增加 MaixCam 帧计数。
// ============================================================

#define TAKEOFF_HIGHT_CM                 100
#define HIGHT_ARRIVE_TH_CM               10
#define POS_ARRIVE_TH_CM                 8
#define YAW_ARRIVE_TH_DEG                5.0f

#define CAMERA_CH_INDEX                  ch_7_aux3
#define CH_SWITCH_HIGH_TH                1700
#define CH_SWITCH_LOW_TH                 1300

#define CAMERA_TARGET_CODE               0xCA
#define CAMERA_CTRL_FC                   0x50
#define CAMERA_CTRL_CAM                  0x60

// 雷达位置环速度上限。先保守一点，确认安全后再增大。
#define RADAR_NAV_SPEED_LIMIT_CMPS        25
#define RADAR_HOLD_SPEED_LIMIT_CMPS       15

// 相机偏差放缩：相机端暂时不改时，可在这里调。
// 例如相机发 8cm，倍率 3 后，认为二维码相对飞机偏 24cm。
// 建议初始 2~3，不建议直接 10，因为现在它会变成雷达目标点偏移。
#define CAM_POS_SCALE_NUM                1
#define CAM_POS_SCALE_DEN                1

// 接收相机坐标的保护限幅：放缩前超过该值认为异常，忽略。
#define CAM_RAW_ABS_LIMIT_CM             120
// 放缩后目标偏移的保护限幅。
#define CAM_USED_ABS_LIMIT_CM            250

// 雷达目标点滤波。4 表示每次使用 1/4 新观测量，数值越大越平滑。
#define QR_TARGET_FILTER_DIV             4
// 每次有新相机样本时，dis_x_target/dis_y_target 最多向滤波目标移动多少 cm。
#define QR_TARGET_MAX_STEP_CM            12

// 没有新相机样本时的容错周期。UserTask 约20ms一次。
// 75约等于1.5s。超过后不再追最后目标点，改为当前位置悬停。
#define CAM_LOST_HOLD_CYCLES             75

// 相机误差足够小且雷达到达目标后，认为已经在二维码上方。
#define CAM_CENTER_TH_CM                 8
#define QR_ARRIVE_STABLE_CYCLES          60

extern int my_task_flag;
extern uint8_t my_slam_flag;

static uint16_t s_last_step = 0xFFFF;
static int s_stable_cnt = 0;
static int s_qr_arrive_cnt = 0;
static uint8_t s_mission_enable = 0;
static uint8_t s_camera_assist_enable = 0;

static uint8_t s_last_cam_ctrl = 0;
static int32_t s_last_cam_x = 0;
static int32_t s_last_cam_y = 0;
static uint8_t s_cam_sample_inited = 0;
static uint16_t s_cam_lost_cycles = CAM_LOST_HOLD_CYCLES + 1;

static uint8_t s_qr_target_valid = 0;
static int32_t s_qr_target_filt_x = 0;
static int32_t s_qr_target_filt_y = 0;

static int32_t abs_i32(int32_t x)
{
    return (x >= 0) ? x : -x;
}

static int32_t clamp_i32(int32_t v, int32_t min_v, int32_t max_v)
{
    if (v < min_v)
    {
        return min_v;
    }
    if (v > max_v)
    {
        return max_v;
    }
    return v;
}

static int32_t step_to_i32(int32_t cur, int32_t target, int32_t max_step)
{
    int32_t diff = target - cur;

    if (diff > max_step)
    {
        return cur + max_step;
    }
    if (diff < -max_step)
    {
        return cur - max_step;
    }
    return target;
}

static uint8_t step_entered(void)
{
    if (mission_step != s_last_step)
    {
        s_last_step = (uint16_t)mission_step;
        my_task_time_dly_cnt_ms = 0;
        return 1;
    }
    return 0;
}

static uint8_t step_wait_ms(uint32_t wait_ms)
{
    if (my_task_time_dly_cnt_ms < (int)wait_ms)
    {
        my_task_time_dly_cnt_ms += 20;
        return 0;
    }

    my_task_time_dly_cnt_ms = 0;
    return 1;
}

static uint8_t stable_counter_check(uint8_t condition, int *counter, int threshold)
{
    if (condition)
    {
        (*counter)++;
    }
    else
    {
        *counter = 0;
    }

    if (*counter > threshold)
    {
        *counter = 0;
        return 1;
    }
    return 0;
}

static uint8_t hight_arrived_cm(int32_t tar_h, int32_t cur_h, int32_t th_cm)
{
    return (abs_i32(tar_h - cur_h) < th_cm) ? 1u : 0u;
}

static void get_radar_xy(int32_t *cur_x, int32_t *cur_y)
{
    *cur_x = get_corrected_dis_x(dis_x_slam);
    *cur_y = get_corrected_dis_y(dis_y_slam);
}

static uint8_t pos_arrived_radar_cm(int32_t tar_x, int32_t tar_y, int32_t th_cm)
{
    int32_t cur_x;
    int32_t cur_y;

    get_radar_xy(&cur_x, &cur_y);

    return (abs_i32(tar_x - cur_x) < th_cm &&
            abs_i32(tar_y - cur_y) < th_cm) ? 1u : 0u;
}

static void radar_hold_current_position(void)
{
    int32_t cur_x;
    int32_t cur_y;

    get_radar_xy(&cur_x, &cur_y);
    dis_x_target = cur_x;
    dis_y_target = cur_y;
}

static void radar_mode_apply(int32_t speed_limit)
{
    height_target = TAKEOFF_HIGHT_CM;
    yaw_set_hold_target(0);
    pid_set_radar_xy_limit(speed_limit);
    mode_select(4);
}

static uint8_t ch7_camera_switch_want_on(void)
{
    uint16_t ch7 = rc_in.rc_ch.st_data.ch_[CAMERA_CH_INDEX];

    if (ch7 > CH_SWITCH_HIGH_TH && ch7 < 2200)
    {
        return 1;
    }
    if (ch7 < CH_SWITCH_LOW_TH)
    {
        return 0;
    }

    return s_camera_assist_enable;
}

static void camera_assist_reset_state(void)
{
    s_last_cam_ctrl = cam_target_control_flag;
    s_last_cam_x = dis_x_cam_target;
    s_last_cam_y = dis_y_cam_target;
    s_cam_sample_inited = 1;
    s_cam_lost_cycles = CAM_LOST_HOLD_CYCLES + 1;

    s_qr_target_valid = 0;
    s_qr_target_filt_x = dis_x_target;
    s_qr_target_filt_y = dis_y_target;
    s_qr_arrive_cnt = 0;
}

static void camera_assist_enter(void)
{
    s_camera_assist_enable = 1;

    cam_target_code_identity_flag = CAMERA_TARGET_CODE;
    cam_target_control_flag = CAMERA_CTRL_FC;
    dis_x_cam_target = 0;
    dis_y_cam_target = 0;

    radar_hold_current_position();
    radar_mode_apply(RADAR_HOLD_SPEED_LIMIT_CMPS);
    camera_assist_reset_state();

    MY_uart_maixcam_send(CAMERA_TARGET_CODE);   // BB CA FF
    my_send_esp_1_test(0xCA);                   // 调试：已发送相机识别 A 码
}

static void camera_assist_exit(void)
{
    s_camera_assist_enable = 0;

    cam_target_control_flag = CAMERA_CTRL_FC;
    dis_x_cam_target = 0;
    dis_y_cam_target = 0;

    radar_hold_current_position();
    radar_mode_apply(RADAR_HOLD_SPEED_LIMIT_CMPS);
    camera_assist_reset_state();

    my_send_esp_1_test(0x50);                   // 调试：退出视觉辅助
}

static uint8_t camera_has_new_sample(void)
{
    uint8_t ctrl = cam_target_control_flag;
    int32_t cam_x = dis_x_cam_target;
    int32_t cam_y = dis_y_cam_target;

    if (s_cam_sample_inited == 0)
    {
        s_last_cam_ctrl = ctrl;
        s_last_cam_x = cam_x;
        s_last_cam_y = cam_y;
        s_cam_sample_inited = 1;
        return 1;
    }

    if (ctrl != s_last_cam_ctrl || cam_x != s_last_cam_x || cam_y != s_last_cam_y)
    {
        s_last_cam_ctrl = ctrl;
        s_last_cam_x = cam_x;
        s_last_cam_y = cam_y;
        return 1;
    }

    return 0;
}

static uint8_t camera_sample_valid(int32_t cam_x, int32_t cam_y)
{
    if (cam_target_control_flag != CAMERA_CTRL_CAM)
    {
        return 0;
    }

    if (abs_i32(cam_x) > CAM_RAW_ABS_LIMIT_CM || abs_i32(cam_y) > CAM_RAW_ABS_LIMIT_CM)
    {
        return 0;
    }

    return 1;
}

static int32_t scale_cam_pos(int32_t v)
{
    int32_t out;

    out = (v * CAM_POS_SCALE_NUM) / CAM_POS_SCALE_DEN;
    out = clamp_i32(out, -CAM_USED_ABS_LIMIT_CM, CAM_USED_ABS_LIMIT_CM);
    return out;
}

static void update_qr_target_by_camera_sample(int32_t cam_x_raw, int32_t cam_y_raw)
{
    int32_t cur_x;
    int32_t cur_y;
    int32_t cam_x_used;
    int32_t cam_y_used;
    int32_t meas_x;
    int32_t meas_y;

    get_radar_xy(&cur_x, &cur_y);

    // 当前假设：视觉辅助阶段 yaw 锁定为 0，机体系 x/y 与雷达任务坐标系 x/y 基本一致。
    // 若后续 yaw 会变化，需要在这里加入 yaw 旋转变换。
    cam_x_used = scale_cam_pos(cam_x_raw);
    cam_y_used = scale_cam_pos(cam_y_raw);

    meas_x = cur_x + cam_x_used;
    meas_y = cur_y + cam_y_used;

    if (s_qr_target_valid == 0)
    {
        s_qr_target_filt_x = meas_x;
        s_qr_target_filt_y = meas_y;
        s_qr_target_valid = 1;
    }
    else
    {
        s_qr_target_filt_x += (meas_x - s_qr_target_filt_x) / QR_TARGET_FILTER_DIV;
        s_qr_target_filt_y += (meas_y - s_qr_target_filt_y) / QR_TARGET_FILTER_DIV;
    }

    // 雷达目标点平滑移动，避免相机坐标跳变导致飞机突然加速。
    dis_x_target = step_to_i32(dis_x_target, s_qr_target_filt_x, QR_TARGET_MAX_STEP_CM);
    dis_y_target = step_to_i32(dis_y_target, s_qr_target_filt_y, QR_TARGET_MAX_STEP_CM);
}

static void update_camera_assisted_radar_target(void)
{
    uint8_t want_camera = ch7_camera_switch_want_on();
    uint8_t new_sample;
    uint8_t valid_sample;
    int32_t cam_x;
    int32_t cam_y;
    uint8_t radar_ok;
    uint8_t cam_center_ok;

    if (want_camera && !s_camera_assist_enable)
    {
        camera_assist_enter();
    }
    else if (!want_camera && s_camera_assist_enable)
    {
        camera_assist_exit();
    }

    // 无论 CH7 是否开启，始终雷达主控，不进入 mode_select(3)。
    radar_mode_apply((s_camera_assist_enable != 0) ? RADAR_NAV_SPEED_LIMIT_CMPS : RADAR_HOLD_SPEED_LIMIT_CMPS);

    if (!s_camera_assist_enable)
    {
        radar_hold_current_position();
        return;
    }

    new_sample = camera_has_new_sample();
    cam_x = dis_x_cam_target;
    cam_y = dis_y_cam_target;
    valid_sample = camera_sample_valid(cam_x, cam_y);

    if (new_sample && valid_sample)
    {
        s_cam_lost_cycles = 0;
        update_qr_target_by_camera_sample(cam_x, cam_y);
    }
    else
    {
        if (s_cam_lost_cycles < 0xFFFF)
        {
            s_cam_lost_cycles++;
        }

        // 短时间没有新样本：继续飞向最后一次滤波得到的二维码雷达目标点。
        // 长时间没有新样本：认为目标丢失，回到当前位置雷达悬停。
        if (s_cam_lost_cycles > CAM_LOST_HOLD_CYCLES)
        {
            s_qr_target_valid = 0;
            radar_hold_current_position();
            pid_set_radar_xy_limit(RADAR_HOLD_SPEED_LIMIT_CMPS);
        }
    }

    // 到达判断：雷达到目标点 + 相机误差足够接近中心。
    // 此处只是计数，不自动投放/降落，方便先联调观察。
    radar_ok = pos_arrived_radar_cm(dis_x_target, dis_y_target, POS_ARRIVE_TH_CM);
    cam_center_ok = (valid_sample &&
                     abs_i32(scale_cam_pos(cam_x)) < CAM_CENTER_TH_CM &&
                     abs_i32(scale_cam_pos(cam_y)) < CAM_CENTER_TH_CM) ? 1u : 0u;

    if (radar_ok && cam_center_ok)
    {
        if (s_qr_arrive_cnt < 0x7FFF)
        {
            s_qr_arrive_cnt++;
        }
    }
    else
    {
        s_qr_arrive_cnt = 0;
    }

    if (s_qr_arrive_cnt == QR_ARRIVE_STABLE_CYCLES)
    {
        my_send_esp_1_test(0x66); // 调试：二维码上方稳定到达
    }
}

void UserTask_OneKeyCmd(void)
{
    static uint8_t one_key_land_f = 1;
    static uint8_t one_key_mission_f = 0;

    uint8_t pos_ok;
    uint8_t hight_ok;
    uint8_t yaw_ok;

    if (rc_in.no_signal != 0)
    {
        return;
    }

    // CH6 中位：一键降落，并关闭任务/视觉辅助。
    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1400 &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1600)
    {
        if (one_key_land_f == 0)
        {
            one_key_land_f = OneKey_Land();
        }

        s_mission_enable = 0;
        my_task_flag = 0;
        mission_step = 0;
        s_last_step = 0xFFFF;
        s_camera_assist_enable = 0;
        return;
    }
    else
    {
        one_key_land_f = 0;
    }

    // CH6 高位：启动一次任务。启动后任务锁存。
    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
    {
        if (one_key_mission_f == 0 && s_mission_enable == 0)
        {
            one_key_mission_f = 1;
            s_mission_enable = 1;
            my_task_flag = 1;
            mission_step = 1;
            s_last_step = 0xFFFF;
        }
    }
    else
    {
        one_key_mission_f = 0;
    }

    if (s_mission_enable == 0)
    {
        return;
    }

    switch (mission_step)
    {
    case 0:
    {
        my_task_time_dly_cnt_ms = 0;
        s_stable_cnt = 0;
        s_qr_arrive_cnt = 0;
        s_camera_assist_enable = 0;
    }
    break;

    case 1:
    {
        step_entered();
        mission_step += LX_Change_Mode(2);
    }
    break;

    case 2:
    {
        if (step_entered())
        {
            all_data_init();
            s_stable_cnt = 0;
            s_qr_arrive_cnt = 0;
            s_camera_assist_enable = 0;
        }
        mission_step += FC_Unlock();
    }
    break;

    case 3:
    {
        if (step_entered())
        {
            yaw_zero = yaw;
            set_dis_zero();

            dis_x_target = 0;
            dis_y_target = 0;
            height_target = TAKEOFF_HIGHT_CM;
            yaw_set_hold_target(0);

            pid_set_radar_xy_limit(RADAR_HOLD_SPEED_LIMIT_CMPS);
            mode_select(4);
        }

        if (step_wait_ms(1000))
        {
            mission_step++;
        }
    }
    break;

    case 4:
    {
        // 等待起飞后雷达定点悬停稳定。
        if (step_entered())
        {
            dis_x_target = 0;
            dis_y_target = 0;
            height_target = TAKEOFF_HIGHT_CM;
            yaw_set_hold_target(0);
            pid_set_radar_xy_limit(RADAR_HOLD_SPEED_LIMIT_CMPS);
            mode_select(4);
        }

        pos_ok = pos_arrived_radar_cm(dis_x_target, dis_y_target, POS_ARRIVE_TH_CM);
        hight_ok = hight_arrived_cm(height_target, hight, HIGHT_ARRIVE_TH_CM);
        yaw_ok = yaw_arrived(YAW_ARRIVE_TH_DEG);
        // pos_ok = 1;
        // hight_ok = 1;
        // yaw_ok = 1;

        if (stable_counter_check((uint8_t)(pos_ok && hight_ok && yaw_ok), &s_stable_cnt, 20))
        {
            mission_step = 5;
        }
    }
    break;

    case 5:
    {
        // 联调状态：全程雷达主控；CH7 高位启用相机辅助目标点修正，CH7 低位退出。
        update_camera_assisted_radar_target();
    }
    break;

    default:
    {
        radar_hold_current_position();
        height_target = TAKEOFF_HIGHT_CM;
        yaw_set_hold_target(0);
        pid_set_radar_xy_limit(RADAR_HOLD_SPEED_LIMIT_CMPS);
        mode_select(4);
        mission_step = 5;
    }
    break;
    }
}
