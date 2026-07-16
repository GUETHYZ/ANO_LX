#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Ano_Math.h"

#include "my_contrl.h"
#include "my_get_data.h"
#include "my_send_test.h"
#include "my_uart.h"
#include "my_pid.h"

/*
 * 低空无人机智慧救援：最终联调任务版
 *
 * 已融合：
 * 1. CH6 高位启动任务，CH6 中位一键安全降落；
 * 2. 地面站选择二维码后，飞控读取 cam_target_code_identity_flag，并持续转发给 MaixCam；
 * 3. 起飞、雷达定点、建图悬停、随机航线全图巡航；
 * 4. MaixCam 输出 0x60 后，提前结束巡航，进入“雷达主控 + 相机偏差修正目标点”；
 * 5. 二维码上方稳定停留至少 2s，等待相机舵机投放完成并输出 0x50；
 * 6. 使用 my_send_esp_qr_message() 广播二维码类别和坐标；
 * 7. 返航到起点并自动降落。
 *
 * 坐标约定：
 * - 以起飞区中心为原点，单位 cm；
 * - x/y 使用你航点测试中已经验证的雷达任务坐标；
 * - 上报坐标取二维码中心估计坐标，并限制在安全地图范围内。
 */

#define TASK_PERIOD_MS 20

/* 高度：最终联调先保持与稳定版本接近。若需要投放前下降，优先只改 DROP_HEIGHT_CM。 */
#define TAKEOFF_HEIGHT_CM 150
#define SEARCH_HEIGHT_CM TAKEOFF_HEIGHT_CM
#define DROP_HEIGHT_CM TAKEOFF_HEIGHT_CM

#define HEIGHT_ARRIVE_TH_CM 10
#define POS_ARRIVE_TH_CM 10
#define HOME_ARRIVE_TH_CM 7
#define YAW_ARRIVE_TH_DEG 2.0f

/* 速度上限。最后实机时建议先保持保守，确认稳定后再小幅提高 CRUISE/RETURN。 */
#define CRUISE_SPEED_LIMIT_CMPS 25
#define RETURN_SPEED_LIMIT_CMPS 30
#define RADAR_HOLD_SPEED_LIMIT_CMPS 12
#define CAMERA_TRACK_SPEED_LIMIT_CMPS 15

#define TAKEOFF_STABLE_CNT 25  /* 500ms */
#define WAYPOINT_STABLE_CNT 10 /* 200ms */
#define HOME_STABLE_CNT 30     /* 600ms */
#define WAYPOINT_HOLD_MS 1000
#define MAP_BUILD_HOLD_MS 3000
#define HOME_HOLD_BEFORE_LAND_MS 1500

#define CH_SWITCH_HIGH_TH 1700
#define CH_SWITCH_MID_LOW_TH 1400
#define CH_SWITCH_MID_HIGH_TH 1600
#define CH_SWITCH_VALID_MAX 2200

/* 默认最终比赛自动响应相机 0x60；若调试阶段仍想用 CH7 作为视觉允许开关，把这里改成 1。 */
#define CAMERA_TRIGGER_NEED_CH7 0
#define CAMERA_CH_INDEX ch_7_aux3

#define QR_CODE_A 0xCA
#define QR_CODE_B 0xCB
#define QR_CODE_C 0xCC
#define QR_CODE_DEFAULT QR_CODE_A

#define CAMERA_CTRL_FC 0x50
#define CAMERA_CTRL_CAM 0x60

/* 相机偏差处理：采用整数 13/10，相当于 1.3 倍。若出现追过头，先降到 10/10。 */
#define CAM_POS_SCALE_NUM 14
#define CAM_POS_SCALE_DEN 10

#define CAM_RAW_ABS_LIMIT_CM 150
#define CAM_USED_ABS_LIMIT_CM 240
#define CAM_CENTER_RAW_TH_CM 15
#define CAM_LOST_HOLD_MS 1000
#define CAM_CENTER_LOST_GRACE_MS 300  /* 视觉短时丢帧/误发 0x50 时，中心保持计时的容错窗口 */
#define CAMERA_RELEASE_CONFIRM_MS 400 /* 普通 0x50 需要稳定持续一段时间才确认释放 */
#define CAMERA_RELEASE_MAGIC_X_CM 1   /* MaixCam 投掷完成后 0x50 的 x 标记：普通 0x50 为 0 */
#define CAMERA_RELEASE_MAGIC_Y_CM 1   /* MaixCam 投掷完成后 0x50 的 y 标记：普通 0x50 为 0 */
#define QR_TARGET_FILTER_DIV 4
#define QR_TARGET_MAX_STEP_CM 10

/* 投放与上报 */
#define QR_CENTER_HOLD_MIN_MS 2200 /* 规则要求 2s 以上，这里留 200ms 裕量 */
#define CAMERA_APPROACH_TIMEOUT_MS 25000
#define REPORT_REPEAT_COUNT 8
#define REPORT_REPEAT_INTERVAL_MS 120

/* 依据你实测的安全地图范围：x 最大 240~250，y 最大 300。 */
#define MAP_FLY_X_MIN_CM 0
#define MAP_FLY_X_MAX_CM 240
#define MAP_FLY_Y_MIN_CM 0
#define MAP_FLY_Y_MAX_CM 300

#define STEP_IDLE 0
#define STEP_CHANGE_MODE 1
#define STEP_UNLOCK 2
#define STEP_INIT_TAKEOFF 3
#define STEP_WAIT_TAKEOFF 4
#define STEP_MAP_BUILD_HOLD 5
#define STEP_GOTO_WAYPOINT 6
#define STEP_WAYPOINT_HOLD 7
#define STEP_CAMERA_APPROACH 8
#define STEP_REPORT_RESULT 9
#define STEP_RETURN_HOME 10
#define STEP_HOME_HOLD 11
#define STEP_AUTO_LAND 12
#define STEP_FINISH 13

/* ROUTE_FORCE_ID = 0/1/2 强制固定路线；=255 启用伪随机。 */
#define ROUTE_FORCE_ID 255

extern int my_task_flag;
extern uint8_t my_slam_flag;

typedef struct
{
    int16_t x;
    int16_t y;
} waypoint_t;

/* 航线 0：横向 S 型扫描，覆盖下、中、上三层。 */
static const waypoint_t route_0[] =
    {
        {60, 60},
        {120, 60},
        {235, 60},
        {235, 150},
        {120, 150},
        {10, 150},
        {10, 250},
        {120, 250},
        {240, 250},
        {240, 300},
        {120, 300},
        {10, 300}};

/* 航线 1：纵向条带扫描，适合从左到右覆盖。 */
static const waypoint_t route_1[] =
    {
        {0, 60},
        {0, 300},
        {150, 300},
        {150, 60},
        {190, 60},
        {190, 300},
        {240, 300},
        {240, 60}};

/* 航线 2：斜向折线扫描，减少重复路径并覆盖三个二维码可能区域。 */
static const waypoint_t route_2[] =
    {
        {60, 80},
        {240, 120},
        {20, 190},
        {240, 250},
        {10, 300},
        {240, 300}};

#define ROUTE_0_LEN ((uint8_t)(sizeof(route_0) / sizeof(route_0[0])))
#define ROUTE_1_LEN ((uint8_t)(sizeof(route_1) / sizeof(route_1[0])))
#define ROUTE_2_LEN ((uint8_t)(sizeof(route_2) / sizeof(route_2[0])))
#define ROUTE_COUNT 3

static uint16_t s_last_step = 0xFFFF;
static uint8_t s_mission_enable = 0;
static uint8_t s_mission_started_latch = 0;
static uint8_t s_land_cmd_latch = 1;
static uint8_t s_auto_land_cmd_latch = 0;

static uint8_t s_route_id = 0;
static uint8_t s_wp_index = 0;
static uint16_t s_stable_cnt = 0;

static uint8_t s_target_qr_code = QR_CODE_DEFAULT;
static uint16_t s_cam_last_frame_cnt = 0;
static uint16_t s_cam_lost_ms = 0;
static uint16_t s_camera_phase_ms = 0;
static uint16_t s_center_hold_ms = 0;
static uint8_t s_camera_started = 0;
static uint8_t s_camera_lock_seen = 0;
static uint8_t s_camera_release_seen = 0;
static uint8_t s_center_dwell_ok = 0;
static uint8_t s_last_center_sample_ok = 0;
static uint16_t s_center_lost_ms = 0;
static uint16_t s_release_candidate_ms = 0;

static uint8_t s_qr_target_valid = 0;
static int32_t s_qr_target_filt_x = 0;
static int32_t s_qr_target_filt_y = 0;
static int16_t s_qr_report_x = 0;
static int16_t s_qr_report_y = 0;

static uint8_t s_report_repeat_cnt = 0;
static uint16_t s_report_interval_ms = 0;
static uint16_t s_camera_resend_ms = 0;

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

static uint8_t is_valid_qr_type_local(uint8_t code)
{
    return (code == QR_CODE_A || code == QR_CODE_B || code == QR_CODE_C) ? 1u : 0u;
}

static uint8_t step_entered(void)
{
    if (mission_step != s_last_step)
    {
        s_last_step = (uint16_t)mission_step;
        my_task_time_dly_cnt_ms = 0;
        s_stable_cnt = 0;
        return 1;
    }
    return 0;
}

static uint8_t step_wait_ms(uint16_t wait_ms)
{
    if (my_task_time_dly_cnt_ms < (int)wait_ms)
    {
        my_task_time_dly_cnt_ms += TASK_PERIOD_MS;
        return 0;
    }

    my_task_time_dly_cnt_ms = 0;
    return 1;
}

static uint8_t stable_counter_check(uint8_t condition, uint16_t threshold_cnt)
{
    if (condition)
    {
        if (s_stable_cnt < 60000)
        {
            s_stable_cnt++;
        }
    }
    else
    {
        s_stable_cnt = 0;
    }

    if (s_stable_cnt >= threshold_cnt)
    {
        s_stable_cnt = 0;
        return 1;
    }
    return 0;
}

static uint8_t height_arrived_cm(int32_t target_h, int32_t current_h, int32_t th_cm)
{
    return (abs_i32(target_h - current_h) <= th_cm) ? 1u : 0u;
}

static void get_radar_xy(int32_t *current_x, int32_t *current_y)
{
    *current_x = get_corrected_dis_x(dis_x_slam);
    *current_y = get_corrected_dis_y(dis_y_slam);
}

static uint8_t pos_arrived_radar_cm(int32_t target_x, int32_t target_y, int32_t th_cm)
{
    int32_t current_x;
    int32_t current_y;

    get_radar_xy(&current_x, &current_y);

    return (abs_i32(target_x - current_x) <= th_cm &&
            abs_i32(target_y - current_y) <= th_cm)
               ? 1u
               : 0u;
}

static void radar_enter_mode_once(void)
{
    yaw_set_hold_target(0);
    mode_select(4); /* 定高 + 定航向 + 雷达定点 */
}

static void radar_update_target(int32_t x, int32_t y, int32_t height_cm, int32_t speed_limit)
{
    dis_x_target = (int32_t)clamp_i32(x, MAP_FLY_X_MIN_CM, MAP_FLY_X_MAX_CM);
    dis_y_target = (int32_t)clamp_i32(y, MAP_FLY_Y_MIN_CM, MAP_FLY_Y_MAX_CM);
    height_target = height_cm;
    pid_set_radar_xy_limit(speed_limit);
}

static void radar_hold_current_position(int32_t height_cm, int32_t speed_limit)
{
    int32_t current_x;
    int32_t current_y;

    get_radar_xy(&current_x, &current_y);
    radar_update_target(current_x, current_y, height_cm, speed_limit);
}

static uint8_t camera_gate_open(void)
{
#if (CAMERA_TRIGGER_NEED_CH7 == 0)
    return 1;
#else
    uint16_t ch7 = rc_in.rc_ch.st_data.ch_[CAMERA_CH_INDEX];
    return (ch7 > CH_SWITCH_HIGH_TH && ch7 < CH_SWITCH_VALID_MAX) ? 1u : 0u;
#endif
}

static uint8_t get_route_len(uint8_t route_id)
{
    switch (route_id)
    {
    case 0:
        return ROUTE_0_LEN;
    case 1:
        return ROUTE_1_LEN;
    default:
        return ROUTE_2_LEN;
    }
}

static waypoint_t get_waypoint(uint8_t route_id, uint8_t index)
{
    waypoint_t p;

    switch (route_id)
    {
    case 0:
        p = route_0[index % ROUTE_0_LEN];
        break;
    case 1:
        p = route_1[index % ROUTE_1_LEN];
        break;
    default:
        p = route_2[index % ROUTE_2_LEN];
        break;
    }

    return p;
}

static uint8_t select_route_id(void)
{
#if (ROUTE_FORCE_ID == 0)
    return 0;
#elif (ROUTE_FORCE_ID == 1)
    return 1;
#elif (ROUTE_FORCE_ID == 2)
    return 2;
#else
    int32_t seed;

    seed = (int32_t)rc_in.rc_ch.st_data.ch_[ch_1_rol] +
           (int32_t)rc_in.rc_ch.st_data.ch_[ch_2_pit] +
           (int32_t)rc_in.rc_ch.st_data.ch_[ch_4_yaw] +
           (int32_t)yaw +
           (int32_t)hight +
           get_corrected_dis_x(dis_x_slam) +
           get_corrected_dis_y(dis_y_slam);

    if (seed < 0)
    {
        seed = -seed;
    }

    return (uint8_t)(seed % ROUTE_COUNT);
#endif
}

static void reset_rescue_task_state(void)
{
    s_last_step = 0xFFFF;
    s_mission_enable = 0;
    s_mission_started_latch = 0;
    s_auto_land_cmd_latch = 0;

    s_route_id = 0;
    s_wp_index = 0;
    s_stable_cnt = 0;

    s_cam_last_frame_cnt = g_maixcam_valid_frame_cnt;
    s_cam_lost_ms = 0;
    s_camera_phase_ms = 0;
    s_center_hold_ms = 0;
    s_camera_started = 0;
    s_camera_lock_seen = 0;
    s_camera_release_seen = 0;
    s_center_dwell_ok = 0;
    s_last_center_sample_ok = 0;
    s_center_lost_ms = 0;
    s_release_candidate_ms = 0;

    s_qr_target_valid = 0;
    s_qr_target_filt_x = 0;
    s_qr_target_filt_y = 0;
    s_qr_report_x = 0;
    s_qr_report_y = 0;

    s_report_repeat_cnt = 0;
    s_report_interval_ms = 0;
    s_camera_resend_ms = 0;

    my_task_flag = 0;
    mission_step = STEP_IDLE;
}

static uint8_t current_selected_qr_code(void)
{
    if (is_valid_qr_type_local(cam_target_code_identity_flag))
    {
        return cam_target_code_identity_flag;
    }
    return QR_CODE_DEFAULT;
}

static void send_qr_to_camera_periodically(uint16_t period_ms)
{
    if (s_camera_resend_ms == 0)
    {
        MY_uart_maixcam_send(s_target_qr_code);
    }

    s_camera_resend_ms += TASK_PERIOD_MS;
    if (s_camera_resend_ms >= period_ms)
    {
        s_camera_resend_ms = 0;
    }
}

static uint8_t maixcam_has_new_frame(void)
{
    uint16_t cnt = g_maixcam_valid_frame_cnt;

    if (cnt != s_cam_last_frame_cnt)
    {
        s_cam_last_frame_cnt = cnt;
        return 1;
    }
    return 0;
}

static uint8_t camera_code_matches_target(void)
{
    return (g_maixcam_last_code == s_target_qr_code) ? 1u : 0u;
}

static uint8_t camera_triggered_target(void)
{
    if (!camera_gate_open())
    {
        return 0;
    }

    if (cam_target_control_flag == CAMERA_CTRL_CAM &&
        g_maixcam_last_ctrl == CAMERA_CTRL_CAM &&
        camera_code_matches_target())
    {
        return 1;
    }

    return 0;
}

static int32_t scale_cam_pos(int32_t v)
{
    int32_t out;

    out = (v * CAM_POS_SCALE_NUM) / CAM_POS_SCALE_DEN;
    out = clamp_i32(out, -CAM_USED_ABS_LIMIT_CM, CAM_USED_ABS_LIMIT_CM);
    return out;
}

static uint8_t camera_sample_valid(int32_t cam_x, int32_t cam_y)
{
    if (abs_i32(cam_x) > CAM_RAW_ABS_LIMIT_CM || abs_i32(cam_y) > CAM_RAW_ABS_LIMIT_CM)
    {
        return 0;
    }
    return 1;
}

static void update_qr_target_by_camera_sample(int32_t cam_x_raw, int32_t cam_y_raw)
{
    int32_t current_x;
    int32_t current_y;
    int32_t cam_x_used;
    int32_t cam_y_used;
    int32_t measured_x;
    int32_t measured_y;

    get_radar_xy(&current_x, &current_y);

    /* 当前任务全程 yaw 锁定为 0，因此沿用你已飞通的“当前雷达坐标 + 相机机体系偏差”方案。 */
    cam_x_used = scale_cam_pos(cam_x_raw);
    cam_y_used = scale_cam_pos(cam_y_raw);

    measured_x = clamp_i32(current_x + cam_x_used, MAP_FLY_X_MIN_CM, MAP_FLY_X_MAX_CM);
    measured_y = clamp_i32(current_y + cam_y_used, MAP_FLY_Y_MIN_CM, MAP_FLY_Y_MAX_CM);

    if (s_qr_target_valid == 0)
    {
        s_qr_target_filt_x = measured_x;
        s_qr_target_filt_y = measured_y;
        s_qr_target_valid = 1;
    }
    else
    {
        s_qr_target_filt_x += (measured_x - s_qr_target_filt_x) / QR_TARGET_FILTER_DIV;
        s_qr_target_filt_y += (measured_y - s_qr_target_filt_y) / QR_TARGET_FILTER_DIV;
    }

    dis_x_target = step_to_i32(dis_x_target, s_qr_target_filt_x, QR_TARGET_MAX_STEP_CM);
    dis_y_target = step_to_i32(dis_y_target, s_qr_target_filt_y, QR_TARGET_MAX_STEP_CM);

    dis_x_target = clamp_i32(dis_x_target, MAP_FLY_X_MIN_CM, MAP_FLY_X_MAX_CM);
    dis_y_target = clamp_i32(dis_y_target, MAP_FLY_Y_MIN_CM, MAP_FLY_Y_MAX_CM);
}

static void save_qr_report_position(void)
{
    int32_t report_x;
    int32_t report_y;

    if (s_qr_target_valid)
    {
        report_x = s_qr_target_filt_x;
        report_y = s_qr_target_filt_y;
    }
    else
    {
        get_radar_xy(&report_x, &report_y);
    }

    s_qr_report_x = (int16_t)clamp_i32(report_x, MAP_FLY_X_MIN_CM, MAP_FLY_X_MAX_CM);
    s_qr_report_y = (int16_t)clamp_i32(report_y, MAP_FLY_Y_MIN_CM, MAP_FLY_Y_MAX_CM);
}

static void camera_approach_reset_state(void)
{
    int32_t current_x;
    int32_t current_y;

    get_radar_xy(&current_x, &current_y);

    s_camera_started = 1;
    s_camera_lock_seen = 0;
    s_camera_release_seen = 0;
    s_center_dwell_ok = 0;
    s_last_center_sample_ok = 0;
    s_center_lost_ms = 0;
    s_release_candidate_ms = 0;
    s_camera_phase_ms = 0;
    s_center_hold_ms = 0;
    s_cam_lost_ms = 0;

    s_qr_target_valid = 0;
    s_qr_target_filt_x = current_x;
    s_qr_target_filt_y = current_y;

    s_cam_last_frame_cnt = g_maixcam_valid_frame_cnt;
}

/*
 * 判断 MaixCam 的 0x50 是否表示“投掷完成”。
 *
 * 现在相机端约定：
 *   0x50 + (0,0)：初始待机 / 暂时未识别 / 普通丢帧，不代表投掷完成；
 *   0x50 + (1,1)：舵机投掷动作完成后的释放标记。
 *
 * 因此：
 * 1. 只要已经见过目标二维码的 0x60，收到 0x50+(1,1) 就立即认为投掷完成；
 * 2. 为兼容旧相机逻辑，如果飞控自己确认中心停留达标，则普通 0x50 稳定一段时间后也可释放。
 */
static uint8_t camera_release_frame_is_drop_done(void)
{
    if (g_maixcam_last_ctrl != CAMERA_CTRL_FC)
    {
        return 0;
    }

    if (!camera_code_matches_target())
    {
        return 0;
    }

    if (g_maixcam_last_x == CAMERA_RELEASE_MAGIC_X_CM &&
        g_maixcam_last_y == CAMERA_RELEASE_MAGIC_Y_CM)
    {
        return 2; /* 魔术释放帧：一帧即可确认 */
    }

    if (s_center_dwell_ok)
    {
        return 1; /* 普通 0x50：需要持续确认 */
    }

    return 0;
}

static uint8_t update_camera_approach_and_check_done(void)
{
    uint8_t new_frame;
    uint8_t target_60_frame = 0;
    uint8_t center_ok_now = 0;
    uint8_t center_hold_condition = 0;
    uint8_t pos_ok;
    uint8_t height_ok;
    int32_t cam_x = dis_x_cam_target;
    int32_t cam_y = dis_y_cam_target;

    s_camera_phase_ms += TASK_PERIOD_MS;

    new_frame = maixcam_has_new_frame();
    if (new_frame)
    {
        if (g_maixcam_last_ctrl == CAMERA_CTRL_CAM && camera_code_matches_target())
        {
            /* 只有目标二维码的 0x60 才认为相机真正锁定目标。
             * 之后如果短暂出现 0x50，不会直接判定投放完成，而是进入释放候选等待。
             */
            target_60_frame = 1;
            s_camera_lock_seen = 1;
            s_cam_lost_ms = 0;
            s_release_candidate_ms = 0;

            cam_x = g_maixcam_last_x;
            cam_y = g_maixcam_last_y;

            if (camera_sample_valid(cam_x, cam_y))
            {
                update_qr_target_by_camera_sample(cam_x, cam_y);

                if (abs_i32(cam_x) <= CAM_CENTER_RAW_TH_CM &&
                    abs_i32(cam_y) <= CAM_CENTER_RAW_TH_CM)
                {
                    center_ok_now = 1;
                    s_last_center_sample_ok = 1;
                    s_center_lost_ms = 0;
                }
                else
                {
                    s_last_center_sample_ok = 0;
                    s_center_lost_ms = 0;
                }
            }
        }
        else if (g_maixcam_last_ctrl == CAMERA_CTRL_FC)
        {
            /* 0x50 可能有三种情况：
             * 1. 初次收到目标码后的待机 0x50；
             * 2. 视觉过程中短暂丢目标/误发 0x50；
             * 3. 舵机投放完成后的释放 0x50。
             * 因此这里绝不能见到 0x50 就结束，必须等“曾经 0x60 锁定 + 中心停留达标 + 0x50 稳定确认”。
             */
        }
    }

    if (target_60_frame == 0)
    {
        if (s_cam_lost_ms < 60000)
        {
            s_cam_lost_ms += TASK_PERIOD_MS;
        }

        if (s_center_lost_ms < 60000)
        {
            s_center_lost_ms += TASK_PERIOD_MS;
        }
    }

    pos_ok = pos_arrived_radar_cm(dis_x_target, dis_y_target, POS_ARRIVE_TH_CM);
    height_ok = height_arrived_cm(DROP_HEIGHT_CM, hight, HEIGHT_ARRIVE_TH_CM);

    /* 中心停留计时允许短时丢帧：
     * - 当前 0x60 帧中心误差足够小，直接计时；
     * - 刚刚中心对准过，但随后短时间丢帧/0x50，则在容错窗口内继续保持，不立刻清零；
     * - 一旦已经满足 2s 停留要求，置位 s_center_dwell_ok，后续等待相机投放释放即可。
     */
    if (center_ok_now)
    {
        center_hold_condition = 1;
    }
    else if (s_last_center_sample_ok && s_center_lost_ms <= CAM_CENTER_LOST_GRACE_MS)
    {
        center_hold_condition = 1;
    }

    if (s_center_dwell_ok == 0)
    {
        if (pos_ok && height_ok && center_hold_condition)
        {
            if (s_center_hold_ms < 60000)
            {
                s_center_hold_ms += TASK_PERIOD_MS;
            }
            save_qr_report_position();

            if (s_center_hold_ms >= QR_CENTER_HOLD_MIN_MS)
            {
                s_center_dwell_ok = 1;
                s_center_hold_ms = QR_CENTER_HOLD_MIN_MS;
                my_send_esp_1_test(0xD2); /* 调试：二维码上方停留时间达标 */
            }
        }
        else
        {
            s_center_hold_ms = 0;
        }
    }

    if (s_cam_lost_ms > CAM_LOST_HOLD_MS && s_center_dwell_ok == 0)
    {
        /* 未完成 2s 停留前，如果长时间没有目标 0x60，就先悬停在当前位置，避免盲目漂移。 */
        radar_hold_current_position(DROP_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
    }
    else
    {
        /* 已经有二维码估计点时，继续守住最后的目标点；短时丢帧/0x50 不改变目标。 */
        radar_update_target(dis_x_target, dis_y_target, DROP_HEIGHT_CM,
                            s_center_dwell_ok ? RADAR_HOLD_SPEED_LIMIT_CMPS : CAMERA_TRACK_SPEED_LIMIT_CMPS);
    }

    /* 释放确认：
     * - 推荐逻辑：MaixCam 舵机动作完成后发送 0x50+(1,1)，一帧即可确认，避免相机端只发出一帧或短时间发送时飞控错过 400ms 窗口；
     * - 兼容逻辑：普通 0x50+(0,0) 只有在飞控自己确认二维码上方停留达标后，才需要稳定持续 CAMERA_RELEASE_CONFIRM_MS。
     * - 如果中间又出现 0x60，s_release_candidate_ms 会在上方被清零，继续视觉跟踪。
     */
    if (s_camera_lock_seen)
    {
        uint8_t release_type = camera_release_frame_is_drop_done();

        if (release_type == 2)
        {
            s_camera_release_seen = 1;
            my_send_esp_1_test(0xD0); /* 调试：收到 0x50+(1,1)，确认投掷完成 */
        }
        else if (release_type == 1)
        {
            if (s_release_candidate_ms < 60000)
            {
                s_release_candidate_ms += TASK_PERIOD_MS;
            }

            if (s_release_candidate_ms >= CAMERA_RELEASE_CONFIRM_MS)
            {
                s_camera_release_seen = 1;
                my_send_esp_1_test(0xD0); /* 调试：普通 0x50 稳定确认释放 */
            }
        }
    }

    if (s_camera_release_seen)
    {
        save_qr_report_position();
        return 1;
    }

    /* 兜底：避免相机一直不释放导致飞机无限悬停。实机若不需要可把超时时间调大。 */
    if (s_camera_phase_ms >= CAMERA_APPROACH_TIMEOUT_MS && s_qr_target_valid)
    {
        save_qr_report_position();
        my_send_esp_1_test(0xE1); /* 调试：视觉阶段超时，使用当前二维码估计坐标返航 */
        return 1;
    }

    return 0;
}

static void report_qr_result_once(void)
{
    my_send_esp_qr_message(DEVICE_BROADCAST, s_target_qr_code, s_qr_report_x, s_qr_report_y);
}

void UserTask_OneKeyCmd(void)
{
    uint8_t pos_ok;
    uint8_t height_ok;
    uint8_t yaw_ok;
    waypoint_t wp;

    if (rc_in.no_signal != 0)
    {
        return;
    }

    /* CH6 中位：人工安全降落，优先级最高。 */
    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > CH_SWITCH_MID_LOW_TH &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < CH_SWITCH_MID_HIGH_TH)
    {
        if (s_land_cmd_latch == 0)
        {
            height_target = 0;
            s_land_cmd_latch = OneKey_Land();
        }

        reset_rescue_task_state();
        return;
    }
    else
    {
        s_land_cmd_latch = 0;
    }

    /* CH6 高位：启动一次完整救援任务。任务启动后锁存，不需要 CH6 一直保持高位。 */
    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > CH_SWITCH_HIGH_TH &&
        rc_in.rc_ch.st_data.ch_[ch_6_aux2] < CH_SWITCH_VALID_MAX)
    {
        if (s_mission_started_latch == 0 && s_mission_enable == 0)
        {
            s_mission_started_latch = 1;
            s_mission_enable = 1;
            my_task_flag = 1;
            mission_step = STEP_CHANGE_MODE;
            s_last_step = 0xFFFF;
        }
    }
    else
    {
        s_mission_started_latch = 0;
    }

    if (s_mission_enable == 0)
    {
        return;
    }

    switch (mission_step)
    {
    case STEP_IDLE:
    {
        /* 空闲态不做控制。 */
    }
    break;

    case STEP_CHANGE_MODE:
    {
        step_entered();
        mission_step += LX_Change_Mode(2);
    }
    break;

    case STEP_UNLOCK:
    {
        if (step_entered())
        {
            all_data_init();
            s_target_qr_code = current_selected_qr_code();
            cam_target_code_identity_flag = s_target_qr_code;
            MY_uart_maixcam_clear_state();
            MY_uart_maixcam_send(s_target_qr_code);
            s_camera_resend_ms = 0;
            my_send_esp_1_test(s_target_qr_code); /* 调试：确认本次任务目标二维码 */
        }

        mission_step += FC_Unlock();
    }
    break;

    case STEP_INIT_TAKEOFF:
    {
        if (step_entered())
        {
            yaw_zero = yaw;
            set_dis_zero();

            s_route_id = select_route_id();
            s_wp_index = 0;

            s_cam_last_frame_cnt = g_maixcam_valid_frame_cnt;
            s_camera_started = 0;
            s_camera_lock_seen = 0;
            s_camera_release_seen = 0;
            s_center_dwell_ok = 0;
            s_last_center_sample_ok = 0;
            s_center_lost_ms = 0;
            s_release_candidate_ms = 0;
            s_qr_target_valid = 0;

            radar_enter_mode_once();
            radar_update_target(0, 0, TAKEOFF_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);

            my_send_esp_1_test(0xA0 + s_route_id); /* 调试：0xA0/0xA1/0xA2 表示航线 */
        }

        radar_update_target(0, 0, TAKEOFF_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        send_qr_to_camera_periodically(1000);

        if (step_wait_ms(1000))
        {
            mission_step = STEP_WAIT_TAKEOFF;
        }
    }
    break;

    case STEP_WAIT_TAKEOFF:
    {
        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(0, 0, TAKEOFF_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        }

        radar_update_target(0, 0, TAKEOFF_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        send_qr_to_camera_periodically(1000);

        pos_ok = pos_arrived_radar_cm(0, 0, POS_ARRIVE_TH_CM);
        height_ok = height_arrived_cm(TAKEOFF_HEIGHT_CM, hight, HEIGHT_ARRIVE_TH_CM);
        yaw_ok = yaw_arrived(YAW_ARRIVE_TH_DEG);

        if (stable_counter_check((uint8_t)(pos_ok && height_ok && yaw_ok), TAKEOFF_STABLE_CNT))
        {
            mission_step = STEP_MAP_BUILD_HOLD;
        }
    }
    break;

    case STEP_MAP_BUILD_HOLD:
    {
        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(0, 0, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
            my_send_esp_1_test(0x90); /* 调试：开始建图悬停 */
        }

        radar_update_target(0, 0, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        send_qr_to_camera_periodically(1000);

        if (camera_triggered_target())
        {
            mission_step = STEP_CAMERA_APPROACH;
            break;
        }

        if (step_wait_ms(MAP_BUILD_HOLD_MS))
        {
            set_dis_zero();
            radar_update_target(0, 0, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
            my_send_esp_1_test(0x91); /* 调试：建图悬停结束 */
            mission_step = STEP_GOTO_WAYPOINT;
        }
    }
    break;

    case STEP_GOTO_WAYPOINT:
    {
        if (camera_triggered_target())
        {
            mission_step = STEP_CAMERA_APPROACH;
            break;
        }

        if (s_wp_index >= get_route_len(s_route_id))
        {
            mission_step = STEP_RETURN_HOME;
            break;
        }

        wp = get_waypoint(s_route_id, s_wp_index);

        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, CRUISE_SPEED_LIMIT_CMPS);
            my_send_esp_1_test(0xB0 + s_wp_index); /* 调试：当前航点序号 */
        }

        radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, CRUISE_SPEED_LIMIT_CMPS);
        send_qr_to_camera_periodically(1000);

        pos_ok = pos_arrived_radar_cm(wp.x, wp.y, POS_ARRIVE_TH_CM);
        height_ok = height_arrived_cm(SEARCH_HEIGHT_CM, hight, HEIGHT_ARRIVE_TH_CM);
        yaw_ok = yaw_arrived(YAW_ARRIVE_TH_DEG);

        if (stable_counter_check((uint8_t)(pos_ok && height_ok && yaw_ok), WAYPOINT_STABLE_CNT))
        {
            mission_step = STEP_WAYPOINT_HOLD;
        }
    }
    break;

    case STEP_WAYPOINT_HOLD:
    {
        if (camera_triggered_target())
        {
            mission_step = STEP_CAMERA_APPROACH;
            break;
        }

        wp = get_waypoint(s_route_id, s_wp_index);

        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        }

        radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        send_qr_to_camera_periodically(1000);

        if (step_wait_ms(WAYPOINT_HOLD_MS))
        {
            s_wp_index++;

            if (s_wp_index >= get_route_len(s_route_id))
            {
                mission_step = STEP_RETURN_HOME;
            }
            else
            {
                mission_step = STEP_GOTO_WAYPOINT;
            }
        }
    }
    break;

    case STEP_CAMERA_APPROACH:
    {
        if (step_entered())
        {
            radar_enter_mode_once();
            camera_approach_reset_state();
            radar_hold_current_position(DROP_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
            my_send_esp_1_test(0x60); /* 调试：进入视觉辅助定位/投放阶段 */
            my_send_esp_1_test(0x10);
        }

        if (update_camera_approach_and_check_done())
        {
            mission_step = STEP_REPORT_RESULT;
        }
    }
    break;

    case STEP_REPORT_RESULT:
    {
        if (step_entered())
        {
            s_report_repeat_cnt = 0;
            s_report_interval_ms = 0;
            report_qr_result_once();
            s_report_repeat_cnt++;
        }

        radar_update_target(dis_x_target, dis_y_target, DROP_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);

        if (s_report_repeat_cnt < REPORT_REPEAT_COUNT)
        {
            s_report_interval_ms += TASK_PERIOD_MS;
            if (s_report_interval_ms >= REPORT_REPEAT_INTERVAL_MS)
            {
                s_report_interval_ms = 0;
                report_qr_result_once();
                s_report_repeat_cnt++;
            }
        }
        else
        {
            mission_step = STEP_RETURN_HOME;
        }
    }
    break;

    case STEP_RETURN_HOME:
    {
        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(0, 0, SEARCH_HEIGHT_CM, RETURN_SPEED_LIMIT_CMPS);
            my_send_esp_1_test(0xC0); /* 调试：开始返航 */
        }

        radar_update_target(0, 0, SEARCH_HEIGHT_CM, RETURN_SPEED_LIMIT_CMPS);

        pos_ok = pos_arrived_radar_cm(0, 0, HOME_ARRIVE_TH_CM);
        height_ok = height_arrived_cm(SEARCH_HEIGHT_CM, hight, HEIGHT_ARRIVE_TH_CM);
        yaw_ok = yaw_arrived(YAW_ARRIVE_TH_DEG);

        if (stable_counter_check((uint8_t)(pos_ok && height_ok && yaw_ok), HOME_STABLE_CNT))
        {
            mission_step = STEP_HOME_HOLD;
        }
    }
    break;

    case STEP_HOME_HOLD:
    {
        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(0, 0, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        }

        radar_update_target(0, 0, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);

        if (step_wait_ms(HOME_HOLD_BEFORE_LAND_MS))
        {
            mission_step = STEP_AUTO_LAND;
        }
    }
    break;

    case STEP_AUTO_LAND:
    {
        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(0, 0, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
            s_auto_land_cmd_latch = 0;
            my_send_esp_1_test(0xC1); /* 调试：执行自动降落 */
        }

        if (s_auto_land_cmd_latch == 0)
        {
            height_target = 0;
            s_auto_land_cmd_latch = OneKey_Land();
        }

        if (s_auto_land_cmd_latch)
        {
            mission_step = STEP_FINISH;
        }
    }
    break;

    case STEP_FINISH:
    {
        if (step_entered())
        {
            my_send_esp_1_test(0xC2); /* 调试：完整救援任务结束 */
        }

        s_mission_enable = 0;
        my_task_flag = 0;
        mission_step = STEP_IDLE;
    }
    break;

    default:
    {
        /* 异常兜底：进入雷达定点并返航。 */
        radar_enter_mode_once();
        radar_update_target(0, 0, SEARCH_HEIGHT_CM, RETURN_SPEED_LIMIT_CMPS);
        mission_step = STEP_RETURN_HOME;
    }
    break;
    }
}
