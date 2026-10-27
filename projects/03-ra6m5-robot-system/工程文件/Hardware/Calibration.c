#include "Calibration.h"

/*
 * 标定模块实现说明
 *
 * 一、模块边界
 * 1. 本模块只做“相机坐标/图像坐标 -> 机械臂基座坐标”的换算。
 * 2. 本模块不包含机械臂逆运动学，不直接控制 UART，不依赖 Mechanical_hand_control.c 的内部实现。
 * 3. 坐标换算由本模块完成，运动执行由机械臂模块处理。
 *
 * 二、当前支持两种模式
 * 1. 3D 模式：
 *    先用内参和 depth 把 (u, v, depth) 反投影到相机坐标系，再通过 R_cb / t_cb 变换到机械臂基座坐标系。
 * 2. 平面模式：
 *    只对 (u, v) 做二维仿射映射，z 直接使用固定高度宏。
 *
 * 三、建议的标定流程
 * 1. 平面模式：
 *    - 让机械臂末端标记点在工作平面上移动 10~20 个点
 *    - 每个点记录 (u, v) 和机械臂真实 (x_base, y_base)
 *    - 离线拟合二维仿射参数 A11~A23
 *    - 把拟合结果手工写回 CALIB_A11 ~ CALIB_A23
 *    - 把 z 固定成 CALIB_PLANE_Z_FIXED_MM，再用 MANUAL_OFFSET_Z 微调
 *
 * 2. 3D 模式：
 *    - 先完成相机内参标定，得到 fx / fy / cx / cy
 *    - 再确认 depth_scale，把相机 depth 原始值转换成 mm
 *    - 让机械臂夹着一个稳定、可重复检测的标记点移动 10~30 个点
 *    - 每个点记录相机看到的 (u, v, depth) 与机械臂真实 (x_base, y_base, z_base)
 *    - 离线求出相机到基座的刚体变换 R_cb / t_cb
 *    - 把结果手工写回 CALIB_R11 ~ CALIB_TZ_MM
 *    - 最后用 MANUAL_OFFSET_* / TCP_OFFSET_* 做小范围人工微调
 *
 * 四、关于 TCP_OFFSET
 * 1. 当前版本把 TCP_OFFSET 当作基座坐标系下的固定补偿。
 * 2. 这适合第一版快速调通，但它不是严格的工具坐标补偿。
 * 3. 如果后续末端姿态变化较大、抓取方向差异明显，应把 TCP 偏移和末端姿态联动计算。
 */

/*
 * 深度有效性检查
 * 1. 平面模式不依赖 depth，因此不做检查。
 * 2. 3D 模式必须保证 depth 合法，否则反投影无意义。
 */
static int Calibration_CheckDepth(float depth)
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

/*
 * 统一叠加手工补偿
 * 1. MANUAL_OFFSET_* 用于整体误差微调。
 * 2. TCP_OFFSET_* 用于夹爪参考点和真实抓取点之间的简化补偿。
 */
static void Calibration_ApplyOffsets(float *x_base, float *y_base, float *z_base)
{
    *x_base += CALIB_MANUAL_OFFSET_X_MM + CALIB_TCP_OFFSET_X_MM;
    *y_base += CALIB_MANUAL_OFFSET_Y_MM + CALIB_TCP_OFFSET_Y_MM;
    *z_base += CALIB_MANUAL_OFFSET_Z_MM + CALIB_TCP_OFFSET_Z_MM;
}

/*
 * 3D 模式换算
 * 1. 先把 (u, v, depth) 反投影为相机坐标系 Pc。
 * 2. 再通过 R_cb / t_cb 换算到机械臂基座坐标系 Pb。
 * 3. 最后叠加手工补偿与 TCP 补偿。
 */
static int Calibration_ImageToBase3D(float u, float v, float depth,
                                     float *x_base, float *y_base, float *z_base)
{
    float z_cam_mm = 0.0f;
    float x_cam_mm = 0.0f;
    float y_cam_mm = 0.0f;

    if (CALIB_OK != Calibration_CheckDepth(depth))
    {
        return CALIB_ERR_DEPTH;
    }

    z_cam_mm = depth * CALIB_DEPTH_SCALE;
    x_cam_mm = ((u - CALIB_CX) * z_cam_mm) / CALIB_FX;
    y_cam_mm = ((v - CALIB_CY) * z_cam_mm) / CALIB_FY;

    *x_base = (CALIB_R11 * x_cam_mm) + (CALIB_R12 * y_cam_mm) + (CALIB_R13 * z_cam_mm) + CALIB_TX_MM;
    *y_base = (CALIB_R21 * x_cam_mm) + (CALIB_R22 * y_cam_mm) + (CALIB_R23 * z_cam_mm) + CALIB_TY_MM;
    *z_base = (CALIB_R31 * x_cam_mm) + (CALIB_R32 * y_cam_mm) + (CALIB_R33 * z_cam_mm) + CALIB_TZ_MM;

    Calibration_ApplyOffsets(x_base, y_base, z_base);
    return CALIB_OK;
}

/*
 * 平面模式换算
 * 1. 只使用 (u, v) 做二维仿射映射。
 * 2. z 直接使用固定宏值。
 * 3. 这种方式适合目标基本都在同一工作平面上的场景。
 */
static int Calibration_ImageToBasePlane(float u, float v, float depth,
                                        float *x_base, float *y_base, float *z_base)
{
    (void)depth;

    *x_base = (CALIB_A11 * u) + (CALIB_A12 * v) + CALIB_A13;
    *y_base = (CALIB_A21 * u) + (CALIB_A22 * v) + CALIB_A23;
    *z_base = CALIB_PLANE_Z_FIXED_MM;

    Calibration_ApplyOffsets(x_base, y_base, z_base);
    return CALIB_OK;
}

/*
 * 对外统一接口
 * 1. 上层始终调用这一个接口，不关心当前到底是 3D 还是平面模式。
 * 2. 内部通过 CALIB_MODE 宏切换实现。
 */
int Calibration_ImageToBase(float u, float v, float depth,
                            float *x_base, float *y_base, float *z_base)
{
    if ((0 == x_base) || (0 == y_base) || (0 == z_base))
    {
        return CALIB_ERR_PARAM;
    }

#if (CALIB_MODE == CALIB_MODE_3D)
    return Calibration_ImageToBase3D(u, v, depth, x_base, y_base, z_base);
#elif (CALIB_MODE == CALIB_MODE_PLANE)
    return Calibration_ImageToBasePlane(u, v, depth, x_base, y_base, z_base);
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

/*
 * 结构体版本接口
 * 1. 便于你后续直接把串口收到的一帧视觉数据封装成结构体后调用。
 * 2. 逻辑仍然转到统一入口 Calibration_ImageToBase()。
 */
int Calibration_ImageToBaseStruct(const calib_image_point_t *image_point,
                                  calib_base_point_t *base_point)
{
    if ((0 == image_point) || (0 == base_point))
    {
        return CALIB_ERR_PARAM;
    }

    return Calibration_ImageToBase(image_point->u,
                                   image_point->v,
                                   image_point->depth,
                                   &base_point->x,
                                   &base_point->y,
                                   &base_point->z);
}
