#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>

/**
 * @brief  标定模块只负责坐标换算
 *
 * @note   图像/相机坐标换算到机械臂基座坐标。
 * @note   工具坐标系偏移按末端姿态换算到基座坐标。
 * @note   本模块不做逆运动学，也不发送 UART4 舵机命令。
 */

/**
 * @brief  标定模式选择，当前默认平面仿射标定
 */
#define CALIB_MODE_PLANE  (0U)
#define CALIB_MODE_3D     (1U)
#define CALIB_MODE        CALIB_MODE_PLANE

/**
 * @brief  相机内参和深度比例，depth_scale 用于把 depth 原始值换算成 mm
 */
#define CALIB_FX          (588.0123f)
#define CALIB_FY          (588.6400f)
#define CALIB_CX          (339.3865f)
#define CALIB_CY          (233.3830f)
#define CALIB_DEPTH_SCALE (1.0f)

/**
 * @brief  3D 标定模式下允许使用的深度范围，单位 mm
 */
#define CALIB_DEPTH_MIN_MM (30.0f)
#define CALIB_DEPTH_MAX_MM (1500.0f)

/**
 * @brief  相机坐标系到机械臂基座坐标系的外参矩阵
 */
#define CALIB_R11   (0.99880090f)
#define CALIB_R12   (-0.02079327f)
#define CALIB_R13   (-0.04432148f)
#define CALIB_R21   (0.02157609f)
#define CALIB_R22   (-0.62569612f)
#define CALIB_R23   (0.77976845f)
#define CALIB_R31   (-0.04394571f)
#define CALIB_R32   (-0.77978972f)
#define CALIB_R33   (-0.62449721f)

#define CALIB_TX_MM (26.78513430f)
#define CALIB_TY_MM (-60.52442781f)
#define CALIB_TZ_MM (317.13131385f)

/**
 * @brief  平面仿射标定参数，输入为图像像素坐标，输出为基座坐标 mm
 */
#define CALIB_A11                             (0.39806223f)
#define CALIB_A12                             (-0.01117215f)
#define CALIB_A13                             (-118.80529104f)
#define CALIB_A21                             (-0.09451792f)
#define CALIB_A22                             (-0.66081519f)
#define CALIB_A23                             (422.89070213f)
#define CALIB_PLANE_Z_FIXED_MM (60.0f)

/**
 * @brief  标定输出的手工补偿，默认不改变标定结果
 */
#define CALIB_MANUAL_OFFSET_X_MM (0.0f)
#define CALIB_MANUAL_OFFSET_Y_MM (0.0f)
#define CALIB_MANUAL_OFFSET_Z_MM (0.0f)

/**
 * @brief  兼容旧版固定 TCP 补偿；姿态联动工具补偿由机械臂 TCP 接口处理
 */
#define CALIB_TCP_OFFSET_X_MM    (0.0f)
#define CALIB_TCP_OFFSET_Y_MM    (0.0f)
#define CALIB_TCP_OFFSET_Z_MM    (0.0f)

/**
 * @brief  标定模块返回码
 */
typedef enum e_calib_status
{
    CALIB_OK        = 0,
    CALIB_ERR_PARAM = -1,
    CALIB_ERR_DEPTH = -2,
    CALIB_ERR_MODE  = -3
} calib_status_t;

/**
 * @brief  图像点或相机点输入
 */
typedef struct st_calib_image_point
{
    float u;        /* 图像横坐标或像素 u */
    float v;        /* 图像纵坐标或像素 v */
    float depth;    /* 深度原始值，平面模式下不参与计算 */
} calib_image_point_t;

/**
 * @brief  机械臂基座坐标输出，单位 mm
 */
typedef struct st_calib_base_point
{
    float x;
    float y;
    float z;
} calib_base_point_t;

/**
 * @brief  标定对象式 API
 *
 * @note   应用层只通过 calibration 对象调用本模块；旧函数名不再对外暴露。
 */
typedef struct st_calibration_api
{
    /**
     * @brief  将图像点换算为机械臂基座坐标
     *
     * @param u       图像横坐标
     * @param v       图像纵坐标
     * @param depth   深度原始值，平面模式下不参与计算
     * @param x_base  基座 x 输出，单位 mm
     * @param y_base  基座 y 输出，单位 mm
     * @param z_base  基座 z 输出，单位 mm
     * @return int    CALIB_OK 表示换算成功
     */
    int (*image_to_base)(float u, float v, float depth,
                         float *x_base, float *y_base, float *z_base);

    /**
     * @brief  结构体形式的图像点到基座坐标换算
     *
     * @param image_point  图像点输入
     * @param base_point   基座坐标输出
     * @return int         CALIB_OK 表示换算成功
     */
    int (*image_to_base_struct)(const calib_image_point_t *image_point,
                                calib_base_point_t *base_point);

    /**
     * @brief  将工具坐标系偏移按 yaw/alpha 转换到基座坐标系
     *
     * @param yaw_deg     工具前向在水平面内从基座 +Y 转向 +X 的角度
     * @param alpha_deg   工具前向俯仰角，0 度为水平，负值向下
     * @param tool_x_mm   工具坐标 X 偏移，单位 mm
     * @param tool_y_mm   工具坐标 Y 偏移，单位 mm
     * @param tool_z_mm   工具坐标 Z 偏移，单位 mm
     * @param base_x_mm   基座坐标 X 偏移输出，单位 mm
     * @param base_y_mm   基座坐标 Y 偏移输出，单位 mm
     * @param base_z_mm   基座坐标 Z 偏移输出，单位 mm
     * @return int        CALIB_OK 表示换算成功
     */
    int (*tool_offset_to_base)(float yaw_deg, float alpha_deg,
                               float tool_x_mm, float tool_y_mm, float tool_z_mm,
                               float *base_x_mm, float *base_y_mm, float *base_z_mm);
} calibration_api_t;

extern const calibration_api_t calibration;

#endif /* CALIBRATION_H_ */
