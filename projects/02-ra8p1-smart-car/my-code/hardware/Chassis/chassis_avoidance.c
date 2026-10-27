#include "hardware/Chassis/chassis_avoidance.h"

#include "hardware/Chassis/Chassis.h"
#include "hardware/Chassis/encoder.h"
#include "hardware/N10/n10_radar.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/* 雷达安装角度偏移，单位度。 */
#define CHASSIS_AVOIDANCE_RADAR_ANGLE_OFFSET_DEG        (0.0f)

/* 圆周角度，单位度。 */
#define CHASSIS_AVOIDANCE_FULL_CIRCLE_DEG               (360.0f)

/* 半圆角度，单位度。 */
#define CHASSIS_AVOIDANCE_HALF_CIRCLE_DEG               (180.0f)

/* 前向视场左边界，单位度。 */
#define CHASSIS_AVOIDANCE_FOV_LEFT_END_DEG              (70.0f)

/* 前向视场右边界，单位度。 */
#define CHASSIS_AVOIDANCE_FOV_RIGHT_START_DEG           (300.0f)

/* 正前方核心区半宽 ±20，单位度。 */
#define CHASSIS_AVOIDANCE_FRONT_LEFT_DEG                (20.0f)

/* 角度转弧度系数。 */
#define CHASSIS_AVOIDANCE_RAD_PER_DEG                   (0.01745329252f)

/* 弧度转角度系数。 */
#define CHASSIS_AVOIDANCE_DEG_PER_RAD                   (57.295779513f)

/* 圆周弧度。 */
#define CHASSIS_AVOIDANCE_TWO_PI_RAD                    (6.283185307f)

/* 圆周率。 */
#define CHASSIS_AVOIDANCE_PI_RAD                        (3.141592654f)

/* 平移速度判零阈值，单位 m/s。 */
#define CHASSIS_AVOIDANCE_TRANSLATION_EPS_MPS           (0.01f)

/* 雷达参与风险计算的最大距离，单位 mm。 */
#define CHASSIS_AVOIDANCE_DETECT_DISTANCE_MM            (1200U)

/* 触发避障接管的前向障碍距离，单位 mm。 */
#define CHASSIS_AVOIDANCE_DANGER_DISTANCE_MM            (300U)

/* 触发更强减速的前向障碍距离，单位 mm。 */
#define CHASSIS_AVOIDANCE_FRONT_SLOW_DISTANCE_MM        (150U)

/* 触发急停的前向障碍距离，单位 mm。 */
#define CHASSIS_AVOIDANCE_EMERGENCY_DISTANCE_MM         (50U)

/* 避障阶段最大前进速度，单位 m/s。 */
#define CHASSIS_AVOIDANCE_AVOID_MAX_VX_MPS              (0.20f)

/* 避障阶段最大横移速度，单位 m/s。 */
#define CHASSIS_AVOIDANCE_AVOID_MAX_VY_MPS              (0.25f)

/* 避障阶段最小横移速度，单位 m/s。 */
#define CHASSIS_AVOIDANCE_AVOID_MIN_VY_MPS              (0.10f)

/* 左右风险差转换为横移速度的比例系数。 */
#define CHASSIS_AVOIDANCE_RISK_TO_VY_GAIN               (0.40f)

/* 左右风险差死区，小于该值时选择更空的一侧。 */
#define CHASSIS_AVOIDANCE_RISK_DEADBAND                 (0.05f)

/* 前方堵死判定的正前风险阈值。 */
#define CHASSIS_AVOIDANCE_FRONT_BLOCK_RISK              (0.80f)

/* 前方堵死判定的单侧风险阈值。 */
#define CHASSIS_AVOIDANCE_SIDE_BLOCK_RISK               (0.60f)

/* 雷达完整容器最大允许未更新时间，单位 ms。 */
#define CHASSIS_AVOIDANCE_RADAR_STALE_TIMEOUT_MS        (200U)

/* 避障入口默认调用周期，单位 ms。 */
#define CHASSIS_AVOIDANCE_DEFAULT_INTERVAL_MS           (10U)

/* 回轨横向修正最大速度，单位 m/s。 */
#define CHASSIS_AVOIDANCE_RETURN_MAX_SPEED_MPS          (0.15f)

/* 回轨横向误差比例系数。 */
#define CHASSIS_AVOIDANCE_RETURN_KP                     (1.20f)

/* 回轨完成横向误差阈值，单位 m。 */
#define CHASSIS_AVOIDANCE_RETURN_DONE_ERROR_M           (0.03f)

/* 回轨最大持续时间，单位 ms。 */
#define CHASSIS_AVOIDANCE_RETURN_TIMEOUT_MS             (3000U)

typedef struct st_chassis_avoidance_fov_risk
{
    /**
     * @brief  左前区域累计风险
     */
    float left_risk;

    /**
     * @brief  右前区域累计风险
     */
    float right_risk;

    /**
     * @brief  正前区域累计风险
     */
    float front_risk;

    /**
     * @brief  前向视场最近有效距离，单位 mm
     */
    uint16_t nearest_distance_mm;

    /**
     * @brief  前向视场内是否存在有效点
     */
    bool has_valid;
} chassis_avoidance_fov_risk_t;

/* 避障状态机当前状态。 */
static chassis_avoidance_state_t g_chassis_avoidance_state = CHASSIS_AVOIDANCE_STATE_NORMAL;

/* 最近一次使用的完整雷达容器序号。 */
static uint32_t g_chassis_avoidance_last_radar_sequence = 0U;

/* 雷达完整容器未更新累计时间，单位 ms。 */
static uint32_t g_chassis_avoidance_radar_stale_ms = 0U;

/* 当前短时里程计估计 X 坐标，单位 m。 */
static float g_chassis_avoidance_pose_x_m = 0.0f;

/* 当前短时里程计估计 Y 坐标，单位 m。 */
static float g_chassis_avoidance_pose_y_m = 0.0f;

/* 当前短时里程计估计航向角，单位 rad。 */
static float g_chassis_avoidance_pose_yaw_rad = 0.0f;

/* 避障开始时估计 X 坐标，单位 m。 */
static float g_chassis_avoidance_track_start_x_m = 0.0f;

/* 避障开始时估计 Y 坐标，单位 m。 */
static float g_chassis_avoidance_track_start_y_m = 0.0f;

/* 避障开始时原始轨道横向轴 X 分量。 */
static float g_chassis_avoidance_track_lateral_x = 0.0f;

/* 避障开始时原始轨道横向轴 Y 分量。 */
static float g_chassis_avoidance_track_lateral_y = 1.0f;

/* 避障开始时上层期望运动方向，单位度。 */
static float g_chassis_avoidance_original_angle_deg = 0.0f;

/* 回轨状态持续时间，单位 ms。 */
static uint32_t g_chassis_avoidance_return_elapsed_ms = 0U;

/* 避障模块最新状态快照。 */
static chassis_avoidance_status_t g_chassis_avoidance_status =
{
    .state = CHASSIS_AVOIDANCE_STATE_NORMAL,
    .radar_sequence = 0U,
    .radar_stale_ms = 0U,
    .target_angle_deg = 0.0f,
    .selected_angle_deg = 0.0f,
    .nearest_distance_mm = N10_RADAR_CLOUD_INVALID_DISTANCE_MM,
    .lateral_error_m = 0.0f,
    .target_safe = false,
};

/**
 * @brief  将浮点数限制到指定范围
 *
 * @param value  输入值
 * @param low    下限
 * @param high   上限
 * @return float 限幅后的值
 */
static float chassis_avoidance_limit_float(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}

/**
 * @brief  将角度归一化到 0~360 度范围
 *
 * @param angle_deg  输入角度，单位度
 * @return float 归一化角度，单位度
 */
static float chassis_avoidance_normalize_angle(float angle_deg)
{
    while (angle_deg >= CHASSIS_AVOIDANCE_FULL_CIRCLE_DEG)
    {
        angle_deg -= CHASSIS_AVOIDANCE_FULL_CIRCLE_DEG;
    }

    while (angle_deg < 0.0f)
    {
        angle_deg += CHASSIS_AVOIDANCE_FULL_CIRCLE_DEG;
    }

    return angle_deg;
}

/**
 * @brief  将弧度归一化到 -pi~pi 范围
 *
 * @param angle_rad  输入弧度
 * @return float 归一化后的弧度
 */
static float chassis_avoidance_normalize_rad(float angle_rad)
{
    while (angle_rad > CHASSIS_AVOIDANCE_PI_RAD)
    {
        angle_rad -= CHASSIS_AVOIDANCE_TWO_PI_RAD;
    }

    while (angle_rad < -CHASSIS_AVOIDANCE_PI_RAD)
    {
        angle_rad += CHASSIS_AVOIDANCE_TWO_PI_RAD;
    }

    return angle_rad;
}

/**
 * @brief  判断角度是否落在闭区间内
 *
 * @param angle_deg  待判断角度，单位度
 * @param start_deg  区间起始角，单位度
 * @param end_deg    区间结束角，单位度
 * @return bool true 表示落在区间内
 */
static bool chassis_avoidance_angle_in_range(float angle_deg, float start_deg, float end_deg)
{
    float angle = chassis_avoidance_normalize_angle(angle_deg);
    float start = chassis_avoidance_normalize_angle(start_deg);
    float end = chassis_avoidance_normalize_angle(end_deg);

    if (start <= end)
    {
        return ((angle >= start) && (angle <= end));
    }

    return ((angle >= start) || (angle <= end));
}

/**
 * @brief  根据车体速度计算运动方向
 *
 * @param vx_mps  前进速度，单位 m/s
 * @param vy_mps  横移速度，单位 m/s
 * @return float 运动方向，单位度
 */
static float chassis_avoidance_target_angle(float vx_mps, float vy_mps)
{
    float angle_rad = atan2f(vy_mps, vx_mps);
    float angle_deg = angle_rad * CHASSIS_AVOIDANCE_DEG_PER_RAD;

    return chassis_avoidance_normalize_angle(angle_deg);
}

/**
 * @brief  计算前向相对角，左前为正，右前为负
 *
 * @param angle_deg  已归一化角度，单位度
 * @return float -180~180 度范围内的相对角
 */
static float chassis_avoidance_front_relative_angle(float angle_deg)
{
    float relative = chassis_avoidance_normalize_angle(angle_deg);

    if (relative > CHASSIS_AVOIDANCE_HALF_CIRCLE_DEG)
    {
        relative -= CHASSIS_AVOIDANCE_FULL_CIRCLE_DEG;
    }

    return relative;
}

/**
 * @brief  判断角度是否处于前向可信视场
 *
 * @param angle_deg  已归一化角度，单位度
 * @return bool true 表示角度在 270~90 度前向半区内
 */
static bool chassis_avoidance_is_front_fov(float angle_deg)
{
    return chassis_avoidance_angle_in_range(angle_deg,
                                           CHASSIS_AVOIDANCE_FOV_RIGHT_START_DEG,
                                           CHASSIS_AVOIDANCE_FOV_LEFT_END_DEG);
}

/**
 * @brief  判断雷达扇区距离是否可用于避障
 *
 * @param p_sector  雷达完整容器扇区
 * @return bool true 表示距离有效
 */
static bool chassis_avoidance_sector_distance_valid(const n10_radar_cloud_sector_t *p_sector)
{
    if (NULL == p_sector)
    {
        return false;
    }

    if (0U == p_sector->valid_count)
    {
        return false;
    }

    if (0U == p_sector->min_distance_mm)
    {
        return false;
    }

    if (N10_RADAR_CLOUD_INVALID_DISTANCE_MM == p_sector->min_distance_mm)
    {
        return false;
    }

    return true;
}

/**
 * @brief  计算前向视场内单点风险
 *
 * @param relative_angle_deg  前向相对角，单位度
 * @param distance_mm         障碍距离，单位 mm
 * @return float 0~1 范围内的风险权重
 */
static float chassis_avoidance_calculate_risk(float relative_angle_deg, uint16_t distance_mm)
{
    float risk_distance = 0.0f;
    float front_weight = 0.0f;
    float distance = (float)distance_mm;

    if (distance >= (float)CHASSIS_AVOIDANCE_DETECT_DISTANCE_MM)
    {
        return 0.0f;
    }

    risk_distance = ((float)CHASSIS_AVOIDANCE_DETECT_DISTANCE_MM - distance) /
                    (float)CHASSIS_AVOIDANCE_DETECT_DISTANCE_MM;
    risk_distance = chassis_avoidance_limit_float(risk_distance, 0.0f, 1.0f);

    front_weight = 1.0f - (fabsf(relative_angle_deg) / CHASSIS_AVOIDANCE_FOV_LEFT_END_DEG);
    front_weight = chassis_avoidance_limit_float(front_weight, 0.2f, 1.0f);

    return risk_distance * front_weight;
}

/**
 * @brief  清空前向视场风险统计
 *
 * @param p_risk  风险统计输出
 * @return void
 */
static void chassis_avoidance_clear_fov_risk(chassis_avoidance_fov_risk_t *p_risk)
{
    if (NULL == p_risk)
    {
        return;
    }

    p_risk->left_risk = 0.0f;
    p_risk->right_risk = 0.0f;
    p_risk->front_risk = 0.0f;
    p_risk->nearest_distance_mm = N10_RADAR_CLOUD_INVALID_DISTANCE_MM;
    p_risk->has_valid = false;
}

/**
 * @brief  从 360 度完整容器统计前向 FOV-lite 风险
 *
 * @param p_cloud  N10 完整雷达容器
 * @param p_risk   前向风险统计输出
 * @return void
 */
static void chassis_avoidance_build_fov_risk(const n10_radar_cloud_t *p_cloud,
                                             chassis_avoidance_fov_risk_t *p_risk)
{
    uint32_t i = 0U;
    float angle = 0.0f;
    float relative = 0.0f;
    float risk = 0.0f;
    const n10_radar_cloud_sector_t *p_sector = NULL;

    chassis_avoidance_clear_fov_risk(p_risk);

    if ((NULL == p_cloud) || (NULL == p_risk))
    {
        return;
    }

    for (i = 0U; i < N10_RADAR_CLOUD_SECTOR_COUNT; i++)
    {
        p_sector = &p_cloud->sectors[i];
        if (!chassis_avoidance_sector_distance_valid(p_sector))
        {
            continue;
        }

        angle = ((float)i * (float)N10_RADAR_CLOUD_SECTOR_DEG) +
                ((float)N10_RADAR_CLOUD_SECTOR_DEG * 0.5f) +
                CHASSIS_AVOIDANCE_RADAR_ANGLE_OFFSET_DEG;
        angle = chassis_avoidance_normalize_angle(angle);

        if (!chassis_avoidance_is_front_fov(angle))
        {
            continue;
        }

        p_risk->has_valid = true;
        if (p_sector->min_distance_mm < p_risk->nearest_distance_mm)
        {
            p_risk->nearest_distance_mm = p_sector->min_distance_mm;
        }

        if (p_sector->min_distance_mm > CHASSIS_AVOIDANCE_DETECT_DISTANCE_MM)
        {
            continue;
        }

        relative = chassis_avoidance_front_relative_angle(angle);
        risk = chassis_avoidance_calculate_risk(relative, p_sector->min_distance_mm);

        if ((relative >= CHASSIS_AVOIDANCE_FRONT_LEFT_DEG) &&
            (relative <= CHASSIS_AVOIDANCE_FOV_LEFT_END_DEG))
        {
            p_risk->left_risk += risk;
        }
        else if ((relative <= -CHASSIS_AVOIDANCE_FRONT_LEFT_DEG) &&
                 (relative >= -CHASSIS_AVOIDANCE_FOV_LEFT_END_DEG))
        {
            p_risk->right_risk += risk;
        }
        else
        {
            p_risk->front_risk += risk;
            if (relative >= 0.0f)
            {
                p_risk->left_risk += risk * 0.5f;
            }
            else
            {
                p_risk->right_risk += risk * 0.5f;
            }
        }
    }
}

/**
 * @brief  判断前向视场是否安全
 *
 * @param p_risk  前向风险统计
 * @return bool true 表示前向视场无需避障接管
 */
static bool chassis_avoidance_fov_safe(const chassis_avoidance_fov_risk_t *p_risk)
{
    if (NULL == p_risk)
    {
        return false;
    }

    if (!p_risk->has_valid)
    {
        return true;
    }

    return (p_risk->nearest_distance_mm > CHASSIS_AVOIDANCE_DANGER_DISTANCE_MM);
}

/**
 * @brief  判断前方是否已经堵死
 *
 * @param p_risk  前向风险统计
 * @return bool true 表示左右和正前都危险，第一版直接停车等待
 */
static bool chassis_avoidance_fov_blocked(const chassis_avoidance_fov_risk_t *p_risk)
{
    if (NULL == p_risk)
    {
        return false;
    }

    if (!p_risk->has_valid)
    {
        return false;
    }

    if (p_risk->nearest_distance_mm > CHASSIS_AVOIDANCE_FRONT_SLOW_DISTANCE_MM)
    {
        return false;
    }

    return ((p_risk->front_risk >= CHASSIS_AVOIDANCE_FRONT_BLOCK_RISK) &&
            (p_risk->left_risk >= CHASSIS_AVOIDANCE_SIDE_BLOCK_RISK) &&
            (p_risk->right_risk >= CHASSIS_AVOIDANCE_SIDE_BLOCK_RISK));
}

/**
 * @brief  使用轮速估算短时车体位姿
 *
 * @param interval_ms  控制周期，单位 ms
 * @return void
 */
static void chassis_avoidance_update_odometry(uint32_t interval_ms)
{
    float lf = 0.0f;
    float rf = 0.0f;
    float lr = 0.0f;
    float rr = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    float rotation_radius = 0.0f;
    float dt_s = 0.0f;
    float cos_yaw = 0.0f;
    float sin_yaw = 0.0f;

    if (0U == interval_ms)
    {
        return;
    }

    lf = encoder.get_speed(CHASSIS_WHEEL_LEFT_FRONT);
    rf = encoder.get_speed(CHASSIS_WHEEL_RIGHT_FRONT);
    lr = encoder.get_speed(CHASSIS_WHEEL_LEFT_REAR);
    rr = encoder.get_speed(CHASSIS_WHEEL_RIGHT_REAR);

    rotation_radius = CHASSIS_HALF_WHEEL_SPACING_M + CHASSIS_HALF_AXLE_SPACING_M;
    vx = (lf + rf + lr + rr) * 0.25f;
    vy = (lf - rf - lr + rr) * 0.25f;
    wz = ((-lf + rf - lr + rr) * 0.25f) / rotation_radius;

    dt_s = (float)interval_ms / 1000.0f;
    cos_yaw = cosf(g_chassis_avoidance_pose_yaw_rad);
    sin_yaw = sinf(g_chassis_avoidance_pose_yaw_rad);

    g_chassis_avoidance_pose_x_m += ((cos_yaw * vx) - (sin_yaw * vy)) * dt_s;
    g_chassis_avoidance_pose_y_m += ((sin_yaw * vx) + (cos_yaw * vy)) * dt_s;
    g_chassis_avoidance_pose_yaw_rad =
        chassis_avoidance_normalize_rad(g_chassis_avoidance_pose_yaw_rad + (wz * dt_s));
}

/**
 * @brief  记录避障开始前的短时轨道信息
 *
 * @param target_angle_deg  上层原始运动方向，单位度
 * @return void
 */
static void chassis_avoidance_begin_avoid(float target_angle_deg)
{
    float world_angle_rad = 0.0f;

    if (CHASSIS_AVOIDANCE_STATE_AVOID == g_chassis_avoidance_state)
    {
        return;
    }

    g_chassis_avoidance_track_start_x_m = g_chassis_avoidance_pose_x_m;
    g_chassis_avoidance_track_start_y_m = g_chassis_avoidance_pose_y_m;
    g_chassis_avoidance_original_angle_deg = target_angle_deg;

    world_angle_rad = g_chassis_avoidance_pose_yaw_rad +
                      (target_angle_deg * CHASSIS_AVOIDANCE_RAD_PER_DEG);
    g_chassis_avoidance_track_lateral_x = -sinf(world_angle_rad);
    g_chassis_avoidance_track_lateral_y = cosf(world_angle_rad);
    g_chassis_avoidance_return_elapsed_ms = 0U;
}

/**
 * @brief  更新状态快照
 *
 * @param state                当前状态
 * @param radar_sequence       雷达容器序号
 * @param target_angle_deg     目标方向，单位度
 * @param selected_angle_deg   选择方向，单位度
 * @param nearest_distance_mm  最近障碍距离，单位 mm
 * @param target_safe          目标方向是否安全
 * @return void
 */
static void chassis_avoidance_update_status(chassis_avoidance_state_t state,
                                            uint32_t radar_sequence,
                                            float target_angle_deg,
                                            float selected_angle_deg,
                                            uint16_t nearest_distance_mm,
                                            bool target_safe)
{
    g_chassis_avoidance_status.state = state;
    g_chassis_avoidance_status.radar_sequence = radar_sequence;
    g_chassis_avoidance_status.radar_stale_ms = g_chassis_avoidance_radar_stale_ms;
    g_chassis_avoidance_status.target_angle_deg = target_angle_deg;
    g_chassis_avoidance_status.selected_angle_deg = selected_angle_deg;
    g_chassis_avoidance_status.nearest_distance_mm = nearest_distance_mm;
    g_chassis_avoidance_status.target_safe = target_safe;
}

/**
 * @brief  进入安全停车状态
 *
 * @param radar_sequence       雷达容器序号
 * @param target_angle_deg     目标方向，单位度
 * @param nearest_distance_mm  最近障碍距离，单位 mm
 * @return void
 */
static void chassis_avoidance_fail_safe(uint32_t radar_sequence,
                                        float target_angle_deg,
                                        uint16_t nearest_distance_mm)
{
    g_chassis_avoidance_state = CHASSIS_AVOIDANCE_STATE_FAIL_SAFE;
    chassis.stop();
    chassis_avoidance_update_status(g_chassis_avoidance_state,
                                    radar_sequence,
                                    target_angle_deg,
                                    target_angle_deg,
                                    nearest_distance_mm,
                                    false);
}

/**
 * @brief  根据前向风险生成横移避障速度
 *
 * @param vx_mps              上层期望前进速度，单位 m/s
 * @param target_angle_deg    上层期望方向，单位度
 * @param p_risk              前向风险统计
 * @param p_selected_angle    避障横移方向输出，单位度
 * @return void
 */
static void chassis_avoidance_output_avoid_velocity(float vx_mps,
                                                    float target_angle_deg,
                                                    const chassis_avoidance_fov_risk_t *p_risk,
                                                    float *p_selected_angle)
{
    float risk_diff = 0.0f;
    float avoid_vx = 0.0f;
    float avoid_vy = 0.0f;

    if ((NULL == p_risk) || (NULL == p_selected_angle))
    {
        return;
    }

    if (chassis_avoidance_fov_blocked(p_risk))
    {
        chassis.set_velocity(0.0f, 0.0f, 0.0f);
        *p_selected_angle = target_angle_deg;
        return;
    }

    risk_diff = p_risk->left_risk - p_risk->right_risk;
    if (fabsf(risk_diff) < CHASSIS_AVOIDANCE_RISK_DEADBAND)
    {
        /* 正前方风险接近对称时，选择累计风险更小的一侧横移。 */
        risk_diff = (p_risk->left_risk <= p_risk->right_risk) ?
                    -CHASSIS_AVOIDANCE_RISK_DEADBAND :
                    CHASSIS_AVOIDANCE_RISK_DEADBAND;
    }

    /* 本工程坐标系中 vy > 0 为左移：左前风险更高时向右避障。 */
    avoid_vy = -risk_diff * CHASSIS_AVOIDANCE_RISK_TO_VY_GAIN;
    avoid_vy = chassis_avoidance_limit_float(avoid_vy,
                                             -CHASSIS_AVOIDANCE_AVOID_MAX_VY_MPS,
                                             CHASSIS_AVOIDANCE_AVOID_MAX_VY_MPS);

    if ((avoid_vy > 0.0f) && (avoid_vy < CHASSIS_AVOIDANCE_AVOID_MIN_VY_MPS))
    {
        avoid_vy = CHASSIS_AVOIDANCE_AVOID_MIN_VY_MPS;
    }
    else if ((avoid_vy < 0.0f) && (avoid_vy > -CHASSIS_AVOIDANCE_AVOID_MIN_VY_MPS))
    {
        avoid_vy = -CHASSIS_AVOIDANCE_AVOID_MIN_VY_MPS;
    }

    avoid_vx = vx_mps;
    if (avoid_vx > CHASSIS_AVOIDANCE_AVOID_MAX_VX_MPS)
    {
        avoid_vx = CHASSIS_AVOIDANCE_AVOID_MAX_VX_MPS;
    }

    if (p_risk->nearest_distance_mm <= CHASSIS_AVOIDANCE_FRONT_SLOW_DISTANCE_MM)
    {
        avoid_vx = 0.0f;
    }

    *p_selected_angle = (avoid_vy >= 0.0f) ?
                        CHASSIS_AVOIDANCE_FOV_LEFT_END_DEG :
                        CHASSIS_AVOIDANCE_FOV_RIGHT_START_DEG;

    chassis.set_velocity(avoid_vx, avoid_vy, 0.0f);
}

/**
 * @brief  输出回轨修正速度
 *
 * @param vx_mps       上层期望前进速度，单位 m/s
 * @param vy_mps       上层期望横移速度，单位 m/s
 * @param wz_radps     上层期望旋转速度，单位 rad/s
 * @param interval_ms  控制周期，单位 ms
 * @return bool true 表示回轨完成
 */
static bool chassis_avoidance_output_return_velocity(float vx_mps,
                                                     float vy_mps,
                                                     float wz_radps,
                                                     uint32_t interval_ms)
{
    float dx = g_chassis_avoidance_pose_x_m - g_chassis_avoidance_track_start_x_m;
    float dy = g_chassis_avoidance_pose_y_m - g_chassis_avoidance_track_start_y_m;
    float error = 0.0f;
    float correction = 0.0f;
    float cos_yaw = cosf(g_chassis_avoidance_pose_yaw_rad);
    float sin_yaw = sinf(g_chassis_avoidance_pose_yaw_rad);
    float input_world_x = 0.0f;
    float input_world_y = 0.0f;
    float output_world_x = 0.0f;
    float output_world_y = 0.0f;
    float output_body_vx = 0.0f;
    float output_body_vy = 0.0f;

    g_chassis_avoidance_return_elapsed_ms += interval_ms;

    error = (dx * g_chassis_avoidance_track_lateral_x) +
            (dy * g_chassis_avoidance_track_lateral_y);
    g_chassis_avoidance_status.lateral_error_m = error;

    if ((fabsf(error) <= CHASSIS_AVOIDANCE_RETURN_DONE_ERROR_M) ||
        (g_chassis_avoidance_return_elapsed_ms >= CHASSIS_AVOIDANCE_RETURN_TIMEOUT_MS))
    {
        return true;
    }

    correction = chassis_avoidance_limit_float(-error * CHASSIS_AVOIDANCE_RETURN_KP,
                                               -CHASSIS_AVOIDANCE_RETURN_MAX_SPEED_MPS,
                                               CHASSIS_AVOIDANCE_RETURN_MAX_SPEED_MPS);

    input_world_x = (cos_yaw * vx_mps) - (sin_yaw * vy_mps);
    input_world_y = (sin_yaw * vx_mps) + (cos_yaw * vy_mps);
    output_world_x = input_world_x + (g_chassis_avoidance_track_lateral_x * correction);
    output_world_y = input_world_y + (g_chassis_avoidance_track_lateral_y * correction);

    output_body_vx = (cos_yaw * output_world_x) + (sin_yaw * output_world_y);
    output_body_vy = (-sin_yaw * output_world_x) + (cos_yaw * output_world_y);

    chassis.set_velocity(output_body_vx, output_body_vy, wz_radps);
    return false;
}

/**
 * @brief  复位避障状态机和短时里程计
 *
 * @return void
 */
static void chassis_avoidance_reset_impl(void)
{
    g_chassis_avoidance_state = CHASSIS_AVOIDANCE_STATE_NORMAL;
    g_chassis_avoidance_last_radar_sequence = 0U;
    g_chassis_avoidance_radar_stale_ms = 0U;
    g_chassis_avoidance_pose_x_m = 0.0f;
    g_chassis_avoidance_pose_y_m = 0.0f;
    g_chassis_avoidance_pose_yaw_rad = 0.0f;
    g_chassis_avoidance_track_start_x_m = 0.0f;
    g_chassis_avoidance_track_start_y_m = 0.0f;
    g_chassis_avoidance_track_lateral_x = 0.0f;
    g_chassis_avoidance_track_lateral_y = 1.0f;
    g_chassis_avoidance_original_angle_deg = 0.0f;
    g_chassis_avoidance_return_elapsed_ms = 0U;
    g_chassis_avoidance_status.state = CHASSIS_AVOIDANCE_STATE_NORMAL;
    g_chassis_avoidance_status.radar_sequence = 0U;
    g_chassis_avoidance_status.radar_stale_ms = 0U;
    g_chassis_avoidance_status.target_angle_deg = 0.0f;
    g_chassis_avoidance_status.selected_angle_deg = 0.0f;
    g_chassis_avoidance_status.nearest_distance_mm = N10_RADAR_CLOUD_INVALID_DISTANCE_MM;
    g_chassis_avoidance_status.lateral_error_m = 0.0f;
    g_chassis_avoidance_status.target_safe = false;
}

/**
 * @brief  初始化避障底盘入口模块
 *
 * @return fsp_err_t FSP_SUCCESS 表示初始化成功
 */
static fsp_err_t chassis_avoidance_init_impl(void)
{
    chassis_avoidance_reset_impl();
    return FSP_SUCCESS;
}

/**
 * @brief  停止底盘并进入安全状态
 *
 * @return void
 */
static void chassis_avoidance_stop_impl(void)
{
    chassis_avoidance_fail_safe(g_chassis_avoidance_status.radar_sequence,
                                g_chassis_avoidance_status.target_angle_deg,
                                g_chassis_avoidance_status.nearest_distance_mm);
}

/**
 * @brief  带避障仲裁的底盘速度入口
 *
 * @param vx_mps       上层期望前进速度，单位 m/s
 * @param vy_mps       上层期望横移速度，单位 m/s
 * @param wz_radps     上层期望旋转速度，单位 rad/s
 * @param interval_ms  控制调用周期，单位 ms
 * @return fsp_err_t   FSP_SUCCESS 表示仲裁完成
 */
static fsp_err_t chassis_avoidance_control_impl(float vx_mps,
                                                float vy_mps,
                                                float wz_radps,
                                                uint32_t interval_ms)
{
    n10_radar_cloud_t cloud;
    chassis_avoidance_fov_risk_t risk;
    fsp_err_t err = FSP_SUCCESS;
    float target_angle = 0.0f;
    float selected_angle = 0.0f;
    float target_speed = 0.0f;
    bool target_safe = false;
    bool return_done = false;

    if (0U == interval_ms)
    {
        interval_ms = CHASSIS_AVOIDANCE_DEFAULT_INTERVAL_MS;
    }

    chassis_avoidance_update_odometry(interval_ms);

    target_speed = sqrtf((vx_mps * vx_mps) + (vy_mps * vy_mps));
    target_angle = chassis_avoidance_target_angle(vx_mps, vy_mps);
    selected_angle = target_angle;

    err = n10_radar.cloud_read(&cloud);
    if (FSP_SUCCESS != err)
    {
        g_chassis_avoidance_radar_stale_ms += interval_ms;
        chassis_avoidance_fail_safe(0U, target_angle, N10_RADAR_CLOUD_INVALID_DISTANCE_MM);
        return err;
    }

    if (cloud.sequence != g_chassis_avoidance_last_radar_sequence)
    {
        g_chassis_avoidance_last_radar_sequence = cloud.sequence;
        g_chassis_avoidance_radar_stale_ms = 0U;
    }
    else
    {
        g_chassis_avoidance_radar_stale_ms += interval_ms;
    }

    chassis_avoidance_build_fov_risk(&cloud, &risk);

    if ((0U == cloud.sequence) ||
        (g_chassis_avoidance_radar_stale_ms > CHASSIS_AVOIDANCE_RADAR_STALE_TIMEOUT_MS))
    {
        chassis_avoidance_fail_safe(cloud.sequence, target_angle, risk.nearest_distance_mm);
        return FSP_ERR_TIMEOUT;
    }

    if (target_speed < CHASSIS_AVOIDANCE_TRANSLATION_EPS_MPS)
    {
        g_chassis_avoidance_state = CHASSIS_AVOIDANCE_STATE_NORMAL;
        g_chassis_avoidance_status.lateral_error_m = 0.0f;
        chassis.set_velocity(vx_mps, vy_mps, wz_radps);
        chassis_avoidance_update_status(g_chassis_avoidance_state,
                                        cloud.sequence,
                                        target_angle,
                                        selected_angle,
                                        risk.nearest_distance_mm,
                                        true);
        return FSP_SUCCESS;
    }

    if (risk.has_valid &&
        (risk.nearest_distance_mm <= CHASSIS_AVOIDANCE_EMERGENCY_DISTANCE_MM))
    {
        chassis_avoidance_fail_safe(cloud.sequence, target_angle, risk.nearest_distance_mm);
        return FSP_SUCCESS;
    }

    target_safe = chassis_avoidance_fov_safe(&risk);
    if (target_safe)
    {
        if ((CHASSIS_AVOIDANCE_STATE_AVOID == g_chassis_avoidance_state) ||
            (CHASSIS_AVOIDANCE_STATE_RETURN_TRACK == g_chassis_avoidance_state))
        {
            g_chassis_avoidance_state = CHASSIS_AVOIDANCE_STATE_RETURN_TRACK;
            return_done = chassis_avoidance_output_return_velocity(vx_mps, vy_mps, wz_radps, interval_ms);
            if (!return_done)
            {
                selected_angle = (g_chassis_avoidance_status.lateral_error_m >= 0.0f) ?
                                 CHASSIS_AVOIDANCE_FOV_RIGHT_START_DEG :
                                 CHASSIS_AVOIDANCE_FOV_LEFT_END_DEG;
                chassis_avoidance_update_status(g_chassis_avoidance_state,
                                                cloud.sequence,
                                                target_angle,
                                                selected_angle,
                                                risk.nearest_distance_mm,
                                                true);
                return FSP_SUCCESS;
            }
        }

        g_chassis_avoidance_state = CHASSIS_AVOIDANCE_STATE_NORMAL;
        g_chassis_avoidance_return_elapsed_ms = 0U;
        g_chassis_avoidance_status.lateral_error_m = 0.0f;
        chassis.set_velocity(vx_mps, vy_mps, wz_radps);
        chassis_avoidance_update_status(g_chassis_avoidance_state,
                                        cloud.sequence,
                                        target_angle,
                                        selected_angle,
                                        risk.nearest_distance_mm,
                                        true);
        return FSP_SUCCESS;
    }

    chassis_avoidance_begin_avoid(target_angle);
    g_chassis_avoidance_state = CHASSIS_AVOIDANCE_STATE_AVOID;
    chassis_avoidance_output_avoid_velocity(vx_mps, target_angle, &risk, &selected_angle);
    chassis_avoidance_update_status(g_chassis_avoidance_state,
                                    cloud.sequence,
                                    target_angle,
                                    selected_angle,
                                    risk.nearest_distance_mm,
                                    false);

    return FSP_SUCCESS;
}

/**
 * @brief  读取避障模块运行状态
 *
 * @param p_status  状态输出指针
 * @return fsp_err_t FSP_SUCCESS 表示读取成功
 */
static fsp_err_t chassis_avoidance_status_get_impl(chassis_avoidance_status_t *p_status)
{
    if (NULL == p_status)
    {
        return FSP_ERR_INVALID_POINTER;
    }

    *p_status = g_chassis_avoidance_status;
    return FSP_SUCCESS;
}

const chassis_avoidance_api_t chassis_avoidance =
{
    .init = chassis_avoidance_init_impl,
    .control = chassis_avoidance_control_impl,
    .stop = chassis_avoidance_stop_impl,
    .reset = chassis_avoidance_reset_impl,
    .status_get = chassis_avoidance_status_get_impl,
};
