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
 * 校电赛 E 题：红色×投掷区视觉末端引导任务版
 *
 * 分工原则：
 * 1. 飞控只负责飞行控制、雷达航点、定高、定航向、继续航点飞行和最终降落；
 * 2. MaixCam Pro 负责红色×识别、中心保持计时、舵机投放；
 * 3. 飞控收到 MaixCam 的 0x60 后，进入“雷达主控 + 相机偏差修正目标点”；
 * 4. 飞控不直接驱动舵机，也不自行判定投放完成；
 * 5. 只有在“已经进入红色×末端引导 + 曾经稳定收到 0x60 + 相机稳定回发 0x50”后，
 *    才认为相机端已经完成投放并释放控制权；
 * 6. 投放完成后不返航到起点，而是回到原航线继续执行剩余航点，
 *    到达最后一个航点后自动降落。
 *
 * 坐标约定：
 * - 以起飞区中心为原点，单位 cm；
 * - 机头方向为 x 正方向；
 * - 机头左侧为 y 正方向，因此向机头右侧飞通常对应 y 变小；
 * - yaw 全程保持 0，不跟随地面线转弯。
 */

#define TASK_PERIOD_MS 20

/* 高度：最终联调先保持与稳定版本接近。若需要投放前下降，优先只改 DROP_HEIGHT_CM。 */
#define TAKEOFF_HEIGHT_CM 80
#define SEARCH_HEIGHT_CM TAKEOFF_HEIGHT_CM
#define DROP_HEIGHT_CM TAKEOFF_HEIGHT_CM

#define HEIGHT_ARRIVE_TH_CM 10
#define POS_ARRIVE_TH_CM 5
#define HOME_ARRIVE_TH_CM 5
#define YAW_ARRIVE_TH_DEG 3.0f

/* 速度上限。最后实机时建议先保持保守，确认稳定后再小幅提高 CRUISE/RETURN。 */
#define CRUISE_SPEED_LIMIT_CMPS 20
#define RETURN_SPEED_LIMIT_CMPS 20 /* 保留名字兼容旧语义；本版用于终点段速度。 */
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

#define TARGET_RED_X 0xCA
#define TARGET_UNUSED_B 0xCB
#define TARGET_UNUSED_C 0xCC
#define TARGET_CODE_DEFAULT TARGET_RED_X

#define CAMERA_CTRL_FC 0x50
#define CAMERA_CTRL_CAM 0x60
#define CAMERA_CTRL_LINE 0x61
#define TARGET_BLACK_LINE 0xA1

/* 相机偏差处理：采用整数 13/10，相当于 1.3 倍。若出现追过头，先降到 10/10。 */
#define CAM_POS_SCALE_NUM 12
#define CAM_POS_SCALE_DEN 10

#define CAM_RAW_ABS_LIMIT_CM 150
#define CAM_USED_ABS_LIMIT_CM 240
#define CAM_CENTER_RAW_TH_CM 6
#define CAM_LOST_HOLD_MS 1000
#define CAM_CENTER_LOST_GRACE_MS 300  /* 保留兼容旧变量，本版不再用飞控中心停留判定投放 */
#define CAMERA_RELEASE_CONFIRM_MS 400 /* 保留兼容旧变量，本版改用 DROP_FINISH_CONFIRM_FRAMES */
#define TARGET_FILTER_DIV 4
#define TARGET_MAX_STEP_CM 10

#define LINE_TIMEOUT_MS 400
#define RED_GUIDE_TIMEOUT_MS 500
#define RED_ENTER_CONFIRM_FRAMES 3
#define DROP_FINISH_CONFIRM_FRAMES 6
#define VISION_TASK_FILTER_DIV 4

/*
 * 投放完成后的航点恢复策略。
 * 注意：当前 route_0 实际只有 5 个点：
 *   index 0 = (160, 0)
 *   index 1 = (160, -80)
 *   index 2 = (160, -160)
 *   index 3 = (160, -160)
 *   index 4 = (0, -160)
 * 之前把恢复点写成 index 7 会越界，导致状态机直接进入 STEP_ROUTE_END_GOTO，
 * 表现为投放后直接飞到最后一个点 (0, -160)。
 *
 * 本版采用“按坐标寻找恢复点”的方式：投放后优先飞向 (160, -160)，
 * 如果路线中有重复点，选择最后一个匹配点，即当前 route_0 的 index 3。
 */
#define DROP_RESUME_WP_AUTO 0xFF
#define DROP_RESUME_BY_COORD 0xFE
#define ROUTE0_DROP_DONE_RESUME_WP_INDEX DROP_RESUME_BY_COORD
#define ROUTE1_DROP_DONE_RESUME_WP_INDEX DROP_RESUME_WP_AUTO
#define ROUTE2_DROP_DONE_RESUME_WP_INDEX DROP_RESUME_WP_AUTO
#define ROUTE0_DROP_RESUME_X_CM 160
#define ROUTE0_DROP_RESUME_Y_CM (-160)

/* 投放与上报 */
#define TARGET_CENTER_HOLD_MIN_MS 3000 /* 题目要求投掷区上方悬停 3s 以上，投放由相机端舵机触发。 */
#define CAMERA_APPROACH_TIMEOUT_MS 25000
#define REPORT_REPEAT_COUNT 8
#define REPORT_REPEAT_INTERVAL_MS 100

/* 依据你实测的安全地图范围：x 最大 240~250，y 最大 300。 */
#define MAP_FLY_X_MIN_CM 0
#define MAP_FLY_X_MAX_CM 240
#define MAP_FLY_Y_MIN_CM -300
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
#define STEP_ROUTE_END_GOTO 10
#define STEP_ROUTE_END_HOLD 11
#define STEP_AUTO_LAND 12
#define STEP_FINISH 13

/* ROUTE_FORCE_ID = 0/1/2 强制固定路线；=255 启用伪随机。初测建议固定 0，避免随机路线影响排查。 */
#define ROUTE_FORCE_ID 0

extern int my_task_flag;
extern uint8_t my_slam_flag;

typedef struct
{
    int16_t x;
    int16_t y;
} waypoint_t;

/* 航线 0：按照“先前飞、再右飞、最后后飞”的 yaw 固定路线。
 * 坐标约定：x 正=机头前方，y 正=机头左侧，因此“向右飞”通常是 y 变小。
 * 下面只是初始测试航点，实际比赛应按雷达坐标系重新量尺修正。
 */
static const waypoint_t route_0[] =
    {
        {160, 0},
        {160, -78},
        {160, -160},
        {0, -160}};

/* 航线 1：更保守的短路径，用于低速调试。 */
static const waypoint_t route_1[] =
    {
        {40, 0},
        {80, 0},
        {120, 0},
        {120, -40},
        {120, -80},
        {80, -80},
        {40, -80}};

/* 航线 2：备用覆盖路线。 */
static const waypoint_t route_2[] =
    {
        {50, 0},
        {100, 0},
        {150, 0},
        {150, -60},
        {100, -60},
        {50, -60},
        {50, -120},
        {100, -120},
        {150, -120}};

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
static uint8_t s_resume_wp_index = 0;
static uint8_t s_drop_done = 0;
static uint16_t s_stable_cnt = 0;

static uint8_t s_target_code = TARGET_CODE_DEFAULT;
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

static uint16_t s_line_last_frame_cnt = 0;
static uint16_t s_red_last_frame_cnt = 0;
static uint16_t s_ctrl_last_frame_cnt = 0;
static uint16_t s_line_lost_ms = 0;
static uint16_t s_red_lost_ms = 0;
static uint8_t s_red_enter_valid_count = 0;
static uint8_t s_red_valid_count = 0;
static uint8_t s_drop_finish_count = 0;
static uint8_t s_red_timeout_reported = 0;
static int32_t s_line_y_filt = 0;
static int32_t s_red_x_filt = 0;
static int32_t s_red_y_filt = 0;

static uint8_t s_target_valid = 0;
static int32_t s_target_filt_x = 0;
static int32_t s_target_filt_y = 0;
static int16_t s_target_report_x = 0;
static int16_t s_target_report_y = 0;

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

static uint8_t is_valid_target_code_local(uint8_t code)
{
    return (code == TARGET_RED_X || code == TARGET_UNUSED_B || code == TARGET_UNUSED_C) ? 1u : 0u;
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

static waypoint_t get_last_waypoint(uint8_t route_id)
{
    uint8_t len = get_route_len(route_id);

    if (len == 0)
    {
        waypoint_t p = {0, 0};
        return p;
    }

    return get_waypoint(route_id, (uint8_t)(len - 1));
}

static uint8_t find_last_waypoint_index_by_xy(uint8_t route_id, int16_t x, int16_t y)
{
    uint8_t i;
    uint8_t len = get_route_len(route_id);
    uint8_t found = DROP_RESUME_WP_AUTO;

    for (i = 0; i < len; i++)
    {
        waypoint_t p = get_waypoint(route_id, i);
        if (p.x == x && p.y == y)
        {
            /* 如果存在重复航点，保留最后一个匹配点，避免在同一坐标重复悬停太久。 */
            found = i;
        }
    }

    return found;
}

static uint8_t get_drop_done_resume_wp_index(void)
{
    uint8_t len = get_route_len(s_route_id);
    uint8_t resume = s_resume_wp_index;

    if (len == 0)
    {
        return 0;
    }

    /* route_0：投放完成后明确飞向 (160, -160)，不要再使用旧的 index 7。 */
    if (s_route_id == 0)
    {
        if (ROUTE0_DROP_DONE_RESUME_WP_INDEX == DROP_RESUME_BY_COORD)
        {
            uint8_t idx = find_last_waypoint_index_by_xy(s_route_id,
                                                         ROUTE0_DROP_RESUME_X_CM,
                                                         ROUTE0_DROP_RESUME_Y_CM);
            if (idx != DROP_RESUME_WP_AUTO)
            {
                resume = idx;
            }
        }
        else if (ROUTE0_DROP_DONE_RESUME_WP_INDEX != DROP_RESUME_WP_AUTO)
        {
            resume = ROUTE0_DROP_DONE_RESUME_WP_INDEX;
        }
    }
    else if (s_route_id == 1 && ROUTE1_DROP_DONE_RESUME_WP_INDEX != DROP_RESUME_WP_AUTO)
    {
        resume = ROUTE1_DROP_DONE_RESUME_WP_INDEX;
    }
    else if (s_route_id == 2 && ROUTE2_DROP_DONE_RESUME_WP_INDEX != DROP_RESUME_WP_AUTO)
    {
        resume = ROUTE2_DROP_DONE_RESUME_WP_INDEX;
    }

    /* 关键保护：恢复点不能等于/超过 len，否则会直接进入 STEP_ROUTE_END_GOTO。 */
    if (resume >= len)
    {
        resume = (uint8_t)(len - 1);
    }

    return resume;
}

static void enter_camera_approach_from_route(uint8_t resume_wp_index)
{
    s_resume_wp_index = resume_wp_index;
    mission_step = STEP_CAMERA_APPROACH;
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
    s_resume_wp_index = 0;
    s_drop_done = 0;
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

    s_line_last_frame_cnt = g_maixcam_line_frame_cnt;
    s_red_last_frame_cnt = g_maixcam_red_frame_cnt;
    s_ctrl_last_frame_cnt = g_maixcam_ctrl_frame_cnt;
    s_line_lost_ms = LINE_TIMEOUT_MS + TASK_PERIOD_MS;
    s_red_lost_ms = RED_GUIDE_TIMEOUT_MS + TASK_PERIOD_MS;
    s_red_enter_valid_count = 0;
    s_red_valid_count = 0;
    s_drop_finish_count = 0;
    s_red_timeout_reported = 0;
    s_line_y_filt = 0;
    s_red_x_filt = 0;
    s_red_y_filt = 0;
    vision_control_set_none();

    s_target_valid = 0;
    s_target_filt_x = 0;
    s_target_filt_y = 0;
    s_target_report_x = 0;
    s_target_report_y = 0;

    s_report_repeat_cnt = 0;
    s_report_interval_ms = 0;
    s_camera_resend_ms = 0;

    my_task_flag = 0;
    mission_step = STEP_IDLE;
}

static uint8_t current_selected_target_code(void)
{
    /* 本题只有一个红色×投掷区，不再依赖地面站选择目标类型。
     * 仍然使用 0xCA，是为了兼容 my_uart.c 中已有的合法 code 判断。
     */
    return TARGET_CODE_DEFAULT;
}

static void send_target_to_camera_periodically(uint16_t period_ms)
{
    if (s_camera_resend_ms == 0)
    {
        MY_uart_maixcam_send(s_target_code);
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
    return (g_maixcam_last_code == s_target_code) ? 1u : 0u;
}

static uint8_t camera_triggered_target(void)
{
    Vision_Frame_t red_frame;

    if (s_drop_done)
    {
        return 0;
    }

    if (!camera_gate_open())
    {
        s_red_enter_valid_count = 0;
        return 0;
    }

    if (g_maixcam_red_frame_cnt != s_red_last_frame_cnt)
    {
        s_red_last_frame_cnt = g_maixcam_red_frame_cnt;
        red_frame = latest_red_frame;

        if (red_frame.valid && red_frame.code == s_target_code && red_frame.ctrl == CAMERA_CTRL_CAM)
        {
            if (s_red_enter_valid_count < 255)
            {
                s_red_enter_valid_count++;
            }
        }
    }

    /* 如果巡航阶段持续收到 0x50，说明相机还没锁定红色×，不能触发投放状态。 */
    if (g_maixcam_ctrl_frame_cnt != s_ctrl_last_frame_cnt)
    {
        Vision_Frame_t ctrl_frame = latest_ctrl_frame;
        s_ctrl_last_frame_cnt = g_maixcam_ctrl_frame_cnt;
        if (ctrl_frame.valid && ctrl_frame.code == s_target_code && ctrl_frame.ctrl == CAMERA_CTRL_FC)
        {
            s_red_enter_valid_count = 0;
        }
    }

    if (s_red_enter_valid_count >= RED_ENTER_CONFIRM_FRAMES)
    {
        s_red_enter_valid_count = 0;
        return 1;
    }

    return 0;
}

static void update_line_assist_for_cruise(void)
{
    Vision_Frame_t line_frame;

    if (g_maixcam_line_frame_cnt != s_line_last_frame_cnt)
    {
        s_line_last_frame_cnt = g_maixcam_line_frame_cnt;
        line_frame = latest_line_frame;

        if (line_frame.valid && line_frame.code == TARGET_BLACK_LINE && line_frame.ctrl == CAMERA_CTRL_LINE)
        {
            if (s_line_lost_ms > LINE_TIMEOUT_MS)
            {
                s_line_y_filt = line_frame.y_cm;
            }
            else
            {
                s_line_y_filt += ((int32_t)line_frame.y_cm - s_line_y_filt) / VISION_TASK_FILTER_DIV;
            }
            s_line_lost_ms = 0;
        }
    }
    else
    {
        if (s_line_lost_ms < 60000)
        {
            s_line_lost_ms += TASK_PERIOD_MS;
        }
    }

    if (s_line_lost_ms <= LINE_TIMEOUT_MS)
    {
        vision_control_set_line(s_line_y_filt);
    }
    else
    {
        vision_control_set_none();
        s_line_y_filt = 0;
    }
}

static void disable_line_assist(void)
{
    if (vision_assist_mode == VISION_ASSIST_LINE)
    {
        vision_control_set_none();
    }
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

static void update_red_x_target_by_camera_sample(int32_t cam_x_raw, int32_t cam_y_raw)
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

    if (s_target_valid == 0)
    {
        s_target_filt_x = measured_x;
        s_target_filt_y = measured_y;
        s_target_valid = 1;
    }
    else
    {
        s_target_filt_x += (measured_x - s_target_filt_x) / TARGET_FILTER_DIV;
        s_target_filt_y += (measured_y - s_target_filt_y) / TARGET_FILTER_DIV;
    }

    dis_x_target = step_to_i32(dis_x_target, s_target_filt_x, TARGET_MAX_STEP_CM);
    dis_y_target = step_to_i32(dis_y_target, s_target_filt_y, TARGET_MAX_STEP_CM);

    dis_x_target = clamp_i32(dis_x_target, MAP_FLY_X_MIN_CM, MAP_FLY_X_MAX_CM);
    dis_y_target = clamp_i32(dis_y_target, MAP_FLY_Y_MIN_CM, MAP_FLY_Y_MAX_CM);
}

static void update_red_x_report_estimate(int32_t cam_x_raw, int32_t cam_y_raw)
{
    int32_t current_x;
    int32_t current_y;
    int32_t cam_x_used;
    int32_t cam_y_used;
    int32_t measured_x;
    int32_t measured_y;

    get_radar_xy(&current_x, &current_y);

    cam_x_used = scale_cam_pos(cam_x_raw);
    cam_y_used = scale_cam_pos(cam_y_raw);

    measured_x = clamp_i32(current_x + cam_x_used, MAP_FLY_X_MIN_CM, MAP_FLY_X_MAX_CM);
    measured_y = clamp_i32(current_y + cam_y_used, MAP_FLY_Y_MIN_CM, MAP_FLY_Y_MAX_CM);

    if (s_target_valid == 0)
    {
        s_target_filt_x = measured_x;
        s_target_filt_y = measured_y;
        s_target_valid = 1;
    }
    else
    {
        s_target_filt_x += (measured_x - s_target_filt_x) / TARGET_FILTER_DIV;
        s_target_filt_y += (measured_y - s_target_filt_y) / TARGET_FILTER_DIV;
    }
}

static void save_target_report_position(void)
{
    int32_t report_x;
    int32_t report_y;

    if (s_target_valid)
    {
        report_x = s_target_filt_x;
        report_y = s_target_filt_y;
    }
    else
    {
        get_radar_xy(&report_x, &report_y);
    }

    s_target_report_x = (int16_t)clamp_i32(report_x, MAP_FLY_X_MIN_CM, MAP_FLY_X_MAX_CM);
    s_target_report_y = (int16_t)clamp_i32(report_y, MAP_FLY_Y_MIN_CM, MAP_FLY_Y_MAX_CM);
}

static void camera_approach_reset_state(void)
{
    int32_t current_x;
    int32_t current_y;
    Vision_Frame_t red_frame;

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

    s_red_valid_count = 0;
    s_drop_finish_count = 0;
    s_red_timeout_reported = 0;
    s_red_lost_ms = 0;
    s_line_lost_ms = LINE_TIMEOUT_MS + TASK_PERIOD_MS;
    s_line_y_filt = 0;

    s_target_valid = 0;
    s_target_filt_x = current_x;
    s_target_filt_y = current_y;

    s_red_last_frame_cnt = g_maixcam_red_frame_cnt;
    s_ctrl_last_frame_cnt = g_maixcam_ctrl_frame_cnt;
    s_line_last_frame_cnt = g_maixcam_line_frame_cnt;

    red_frame = latest_red_frame;
    if (red_frame.valid && red_frame.code == s_target_code && red_frame.ctrl == CAMERA_CTRL_CAM)
    {
        s_camera_lock_seen = 1;
        s_red_valid_count = RED_ENTER_CONFIRM_FRAMES;
        s_red_x_filt = red_frame.x_cm;
        s_red_y_filt = red_frame.y_cm;
        update_red_x_report_estimate(red_frame.x_cm, red_frame.y_cm);
        vision_control_set_red(s_red_x_filt, s_red_y_filt);
    }
    else
    {
        s_red_x_filt = 0;
        s_red_y_filt = 0;
        vision_control_set_none();
    }
}

static uint8_t update_camera_approach_and_check_done(void)
{
    uint8_t got_red_60 = 0;
    uint8_t got_release_50 = 0;
    Vision_Frame_t red_frame;
    Vision_Frame_t ctrl_frame;

    s_camera_phase_ms += TASK_PERIOD_MS;

    /* 红色×末端引导阶段：普通航点推进暂停，雷达只负责定住当前位置/定高/定航向。 */
    radar_hold_current_position(DROP_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);

    if (g_maixcam_red_frame_cnt != s_red_last_frame_cnt)
    {
        s_red_last_frame_cnt = g_maixcam_red_frame_cnt;
        red_frame = latest_red_frame;

        if (red_frame.valid && red_frame.code == s_target_code && red_frame.ctrl == CAMERA_CTRL_CAM)
        {
            got_red_60 = 1;
            s_camera_lock_seen = 1;
            s_red_lost_ms = 0;
            s_drop_finish_count = 0;
            s_red_timeout_reported = 0;

            if (s_red_valid_count < 255)
            {
                s_red_valid_count++;
            }

            if (camera_sample_valid(red_frame.x_cm, red_frame.y_cm))
            {
                if (s_red_valid_count <= RED_ENTER_CONFIRM_FRAMES)
                {
                    s_red_x_filt = red_frame.x_cm;
                    s_red_y_filt = red_frame.y_cm;
                }
                else
                {
                    s_red_x_filt += ((int32_t)red_frame.x_cm - s_red_x_filt) / VISION_TASK_FILTER_DIV;
                    s_red_y_filt += ((int32_t)red_frame.y_cm - s_red_y_filt) / VISION_TASK_FILTER_DIV;
                }

                update_red_x_report_estimate(red_frame.x_cm, red_frame.y_cm);
                vision_control_set_red(s_red_x_filt, s_red_y_filt);
            }
        }
    }

    if (got_red_60 == 0)
    {
        if (s_red_lost_ms < 60000)
        {
            s_red_lost_ms += TASK_PERIOD_MS;
        }
    }

    if (g_maixcam_ctrl_frame_cnt != s_ctrl_last_frame_cnt)
    {
        s_ctrl_last_frame_cnt = g_maixcam_ctrl_frame_cnt;
        ctrl_frame = latest_ctrl_frame;

        if (ctrl_frame.valid && ctrl_frame.code == s_target_code && ctrl_frame.ctrl == CAMERA_CTRL_FC)
        {
            got_release_50 = 1;
        }
    }

    /* 只有进入红色×末端引导、且已经稳定见过若干帧 0x60 后，连续 0x50 才能表示投放完成。 */
    if (got_release_50 && s_camera_lock_seen && s_red_valid_count >= RED_ENTER_CONFIRM_FRAMES)
    {
        if (s_drop_finish_count < 255)
        {
            s_drop_finish_count++;
        }

        if (s_drop_finish_count >= DROP_FINISH_CONFIRM_FRAMES)
        {
            s_camera_release_seen = 1;
            save_target_report_position();
            vision_control_set_none();
            my_send_esp_1_test(0xD0); /* 调试：稳定收到0x50，确认相机端已投放完成 */
            return 1;
        }
    }

    if (s_red_lost_ms > RED_GUIDE_TIMEOUT_MS)
    {
        /* 长时间没有红色× 0x60：清零视觉速度，保持悬停，等待重新锁定或相机投放后的 0x50。 */
        vision_control_set_none();
        radar_hold_current_position(DROP_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);

        if (s_red_timeout_reported == 0)
        {
            s_red_timeout_reported = 1;
            my_send_esp_1_test(0xE2); /* 调试：红色×视觉超时，已清零视觉修正并悬停 */
        }
    }

    return 0;
}

static void report_target_result_once(void)
{
    my_send_esp_qr_message(DEVICE_BROADCAST, s_target_code, s_target_report_x, s_target_report_y);
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
            s_target_code = current_selected_target_code();
            cam_target_code_identity_flag = s_target_code;
            MY_uart_maixcam_clear_state();
            MY_uart_maixcam_send(s_target_code);
            s_camera_resend_ms = 0;
            my_send_esp_1_test(s_target_code); /* 调试：确认本次任务红色×目标 */
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
            s_resume_wp_index = 0;
            s_drop_done = 0;

            s_cam_last_frame_cnt = g_maixcam_valid_frame_cnt;
            s_camera_started = 0;
            s_camera_lock_seen = 0;
            s_camera_release_seen = 0;
            s_center_dwell_ok = 0;
            s_last_center_sample_ok = 0;
            s_center_lost_ms = 0;
            s_release_candidate_ms = 0;
            s_target_valid = 0;
            s_line_last_frame_cnt = g_maixcam_line_frame_cnt;
            s_red_last_frame_cnt = g_maixcam_red_frame_cnt;
            s_ctrl_last_frame_cnt = g_maixcam_ctrl_frame_cnt;
            s_line_lost_ms = LINE_TIMEOUT_MS + TASK_PERIOD_MS;
            s_red_lost_ms = RED_GUIDE_TIMEOUT_MS + TASK_PERIOD_MS;
            s_red_enter_valid_count = 0;
            s_red_valid_count = 0;
            s_drop_finish_count = 0;
            s_red_timeout_reported = 0;
            s_line_y_filt = 0;
            s_red_x_filt = 0;
            s_red_y_filt = 0;
            vision_control_set_none();

            radar_enter_mode_once();
            radar_update_target(0, 0, TAKEOFF_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);

            my_send_esp_1_test(0xA0 + s_route_id); /* 调试：0xA0/0xA1/0xA2 表示航线 */
        }

        radar_update_target(0, 0, TAKEOFF_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        send_target_to_camera_periodically(1000);

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
        send_target_to_camera_periodically(1000);

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
        disable_line_assist();
        send_target_to_camera_periodically(1000);

        if (camera_triggered_target())
        {
            enter_camera_approach_from_route(0);
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
            /* 在飞向当前航点途中发现投掷区：先保存当前索引；真正恢复航点在 STEP_REPORT_RESULT 中统一决定。 */
            enter_camera_approach_from_route(s_wp_index);
            break;
        }

        if (s_wp_index >= get_route_len(s_route_id))
        {
            mission_step = STEP_ROUTE_END_GOTO;
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
        update_line_assist_for_cruise();
        send_target_to_camera_periodically(1000);

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
            /* 在航点悬停阶段发现投掷区：说明当前航点已经到过，投放完成后从下一航点继续。 */
            enter_camera_approach_from_route((uint8_t)(s_wp_index + 1));
            break;
        }

        wp = get_waypoint(s_route_id, s_wp_index);

        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        }

        radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        disable_line_assist();
        send_target_to_camera_periodically(1000);

        if (step_wait_ms(WAYPOINT_HOLD_MS))
        {
            s_wp_index++;

            if (s_wp_index >= get_route_len(s_route_id))
            {
                mission_step = STEP_ROUTE_END_GOTO;
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
            my_send_esp_1_test(0x10); /* 到达投掷区上空，激活喇叭和灯光提醒 */
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
            report_target_result_once();
            s_report_repeat_cnt++;
            my_send_esp_1_test(0xD1); /* 调试：开始广播红色×坐标 */
        }

        radar_update_target(dis_x_target, dis_y_target, DROP_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        vision_control_set_none();

        if (s_report_repeat_cnt < REPORT_REPEAT_COUNT)
        {
            s_report_interval_ms += TASK_PERIOD_MS;
            if (s_report_interval_ms >= REPORT_REPEAT_INTERVAL_MS)
            {
                s_report_interval_ms = 0;
                report_target_result_once();
                s_report_repeat_cnt++;
            }
        }
        else
        {
            /*
             * 投掷完成后锁存，避免红色×仍在视野中导致再次进入投放阶段。
             * 关键修复：不要无条件回到 s_resume_wp_index。
             * 当前 route_0 如果使用旧的 index 7 会越界并直接进入最后航点，
             * 因此通过 get_drop_done_resume_wp_index() 按坐标恢复到 (160, -160)。
             */
            s_drop_done = 1;
            s_wp_index = get_drop_done_resume_wp_index();
            my_send_esp_1_test((uint8_t)(0xE0 + (s_wp_index & 0x0F))); /* 调试：投放后恢复航点，当前 route_0 期望 0xE3 */

            if (s_wp_index >= get_route_len(s_route_id))
            {
                mission_step = STEP_ROUTE_END_GOTO;
            }
            else
            {
                mission_step = STEP_GOTO_WAYPOINT;
            }
        }
    }
    break;

    case STEP_ROUTE_END_GOTO:
    {
        wp = get_last_waypoint(s_route_id);

        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RETURN_SPEED_LIMIT_CMPS);
            // my_send_esp_1_test(0xC0); /* 调试：开始飞向最后一个航点 */
        }

        radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RETURN_SPEED_LIMIT_CMPS);
        update_line_assist_for_cruise();

        pos_ok = pos_arrived_radar_cm(wp.x, wp.y, HOME_ARRIVE_TH_CM);
        height_ok = height_arrived_cm(SEARCH_HEIGHT_CM, hight, HEIGHT_ARRIVE_TH_CM);
        yaw_ok = yaw_arrived(YAW_ARRIVE_TH_DEG);

        if (stable_counter_check((uint8_t)(pos_ok && height_ok && yaw_ok), HOME_STABLE_CNT))
        {
            mission_step = STEP_ROUTE_END_HOLD;
        }
    }
    break;

    case STEP_ROUTE_END_HOLD:
    {
        wp = get_last_waypoint(s_route_id);

        if (step_entered())
        {
            radar_enter_mode_once();
            radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
            my_send_esp_1_test(0xC3); /* 调试：最后一个航点悬停，准备降落 */
        }

        radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
        disable_line_assist();

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
            wp = get_last_waypoint(s_route_id);
            radar_enter_mode_once();
            radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RADAR_HOLD_SPEED_LIMIT_CMPS);
            vision_control_set_none();
            s_auto_land_cmd_latch = 0;
            my_send_esp_1_test(0xC1); /* 调试：在最后一个航点执行自动降落 */
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

        vision_control_set_none();
        s_mission_enable = 0;
        my_task_flag = 0;
        mission_step = STEP_IDLE;
    }
    break;

    default:
    {
        /* 异常兜底：进入雷达定点并飞向最后一个航点。 */
        radar_enter_mode_once();
        vision_control_set_none();
        wp = get_last_waypoint(s_route_id);
        radar_update_target(wp.x, wp.y, SEARCH_HEIGHT_CM, RETURN_SPEED_LIMIT_CMPS);
        mission_step = STEP_ROUTE_END_GOTO;
    }
    break;
    }
}
