#include "hardware/mechanical_arm/Calibration.h"

#include <math.h>

#define CALIB_PI (3.1415926f)

static void calibration_apply_offsets(float *x_base, float *y_base, float *z_base);

#if (CALIB_MODE == CALIB_MODE_3D)
static int calibration_check_depth(float depth);
static int calibration_image_to_base_3d(float u, float v, float depth,
                                     float *x_base, float *y_base, float *z_base);
#endif

#if (CALIB_MODE == CALIB_MODE_PLANE)
static int calibration_image_to_base_plane(float u, float v, float depth,
                                        float *x_base, float *y_base, float *z_base);
#endif

/**
 * @brief  对 3D 标定输入深度做基础有效性检查
 */
#if (CALIB_MODE == CALIB_MODE_3D)
static int calibration_check_depth(float depth)
{
    float depth_mm = depth * CALIB_DEPTH_SCALE;

    if (depth_mm < CALIB_DEPTH_MIN_MM)
    {
        return CALIB_ERR_DEPTH;
    }

    if (depth_mm > CALIB_DEPTH_MAX_MM)
    {
        return CALIB_ERR_DEPTH;
    }

    return CALIB_OK;
}
#endif

/**
 * @brief  统一叠加标定手工补偿和旧版固定 TCP 补偿
 */
static void calibration_apply_offsets(float *x_base, float *y_base, float *z_base)
{
    *x_base += CALIB_MANUAL_OFFSET_X_MM + CALIB_TCP_OFFSET_X_MM;
    *y_base += CALIB_MANUAL_OFFSET_Y_MM + CALIB_TCP_OFFSET_Y_MM;
    *z_base += CALIB_MANUAL_OFFSET_Z_MM + CALIB_TCP_OFFSET_Z_MM;
}

/**
 * @brief  3D 模式：像素点和深度先还原到相机坐标，再通过外参转到基座坐标
 */
#if (CALIB_MODE == CALIB_MODE_3D)
static int calibration_image_to_base_3d(float u, float v, float depth,
                                     float *x_base, float *y_base, float *z_base)
{
    float z_cam_mm;
    float x_cam_mm;
    float y_cam_mm;

    if (CALIB_OK != calibration_check_depth(depth))
    {
        return CALIB_ERR_DEPTH;
    }

    z_cam_mm = depth * CALIB_DEPTH_SCALE;
    x_cam_mm = ((u - CALIB_CX) * z_cam_mm) / CALIB_FX;
    y_cam_mm = ((v - CALIB_CY) * z_cam_mm) / CALIB_FY;

    *x_base = (CALIB_R11 * x_cam_mm) + (CALIB_R12 * y_cam_mm) + (CALIB_R13 * z_cam_mm) + CALIB_TX_MM;
    *y_base = (CALIB_R21 * x_cam_mm) + (CALIB_R22 * y_cam_mm) + (CALIB_R23 * z_cam_mm) + CALIB_TY_MM;
    *z_base = (CALIB_R31 * x_cam_mm) + (CALIB_R32 * y_cam_mm) + (CALIB_R33 * z_cam_mm) + CALIB_TZ_MM;

    calibration_apply_offsets(x_base, y_base, z_base);
    return CALIB_OK;
}
#endif

/**
 * @brief  平面模式：直接使用仿射参数把像素坐标换算成基座 x/y，z 使用固定高度
 */
#if (CALIB_MODE == CALIB_MODE_PLANE)
static int calibration_image_to_base_plane(float u, float v, float depth,
                                        float *x_base, float *y_base, float *z_base)
{
    (void)depth;

    *x_base = (CALIB_A11 * u) + (CALIB_A12 * v) + CALIB_A13;
    *y_base = (CALIB_A21 * u) + (CALIB_A22 * v) + CALIB_A23;
    *z_base = CALIB_PLANE_Z_FIXED_MM;

    calibration_apply_offsets(x_base, y_base, z_base);
    return CALIB_OK;
}
#endif

/**
 * @brief  根据当前 CALIB_MODE 选择平面或 3D 标定链路
 */
static int calibration_image_to_base(float u, float v, float depth,
                            float *x_base, float *y_base, float *z_base)
{
    if ((0 == x_base) || (0 == y_base) || (0 == z_base))
    {
        return CALIB_ERR_PARAM;
    }

#if (CALIB_MODE == CALIB_MODE_3D)
    return calibration_image_to_base_3d(u, v, depth, x_base, y_base, z_base);
#elif (CALIB_MODE == CALIB_MODE_PLANE)
    return calibration_image_to_base_plane(u, v, depth, x_base, y_base, z_base);
#else
    (void)u;
    (void)v;
    (void)depth;
    *x_base = 0.0f;
    *y_base = 0.0f;
    *z_base = 0.0f;
    return CALIB_ERR_MODE;
#endif
}

/**
 * @brief  结构体形式的图像点换算，便于视觉链路直接传递点数据
 */
static int calibration_image_to_base_struct(const calib_image_point_t *image_point,
                                  calib_base_point_t *base_point)
{
    if ((0 == image_point) || (0 == base_point))
    {
        return CALIB_ERR_PARAM;
    }

    return calibration_image_to_base(image_point->u,
                                   image_point->v,
                                   image_point->depth,
                                   &base_point->x,
                                   &base_point->y,
                                   &base_point->z);
}

/*
 * 工具坐标系偏移换算：
 * 1. yaw_deg 表示工具前向在水平面内从基座 +Y 转向 +X 的角度。
 * 2. alpha_deg 表示工具前向的俯仰角，0 度为水平，负值向下。
 * 3. tool_x/y/z 是工具坐标系下的偏移，输出为基座坐标系偏移。
 */
static int calibration_tool_offset_to_base(float yaw_deg, float alpha_deg,
                                 float tool_x_mm, float tool_y_mm, float tool_z_mm,
                                 float *base_x_mm, float *base_y_mm, float *base_z_mm)
{
    float yaw_rad;
    float alpha_rad;
    float sin_yaw;
    float cos_yaw;
    float sin_alpha;
    float cos_alpha;

    if ((0 == base_x_mm) || (0 == base_y_mm) || (0 == base_z_mm))
    {
        return CALIB_ERR_PARAM;
    }

    yaw_rad = yaw_deg * CALIB_PI / 180.0f;
    alpha_rad = alpha_deg * CALIB_PI / 180.0f;

    sin_yaw = sinf(yaw_rad);
    cos_yaw = cosf(yaw_rad);
    sin_alpha = sinf(alpha_rad);
    cos_alpha = cosf(alpha_rad);

    *base_x_mm = (tool_x_mm * sin_yaw * cos_alpha) -
                 (tool_y_mm * cos_yaw) -
                 (tool_z_mm * sin_yaw * sin_alpha);
    *base_y_mm = (tool_x_mm * cos_yaw * cos_alpha) +
                 (tool_y_mm * sin_yaw) -
                 (tool_z_mm * cos_yaw * sin_alpha);
    *base_z_mm = (tool_x_mm * sin_alpha) +
                 (tool_z_mm * cos_alpha);

    return CALIB_OK;
}

/**
 * @brief  标定对外对象，应用层通过 calibration.xxx() 调用
 */
const calibration_api_t calibration =
{
    .image_to_base        = calibration_image_to_base,
    .image_to_base_struct = calibration_image_to_base_struct,
    .tool_offset_to_base  = calibration_tool_offset_to_base,
};
