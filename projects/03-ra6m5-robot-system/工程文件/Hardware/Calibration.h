#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>

/*
 * 标定模块说明
 * 1. 本模块只负责把相机返回的 (u, v, depth) 转成机械臂基座坐标系下的 (x, y, z)。
 * 2. 本模块不负责机械臂逆运动学，不负责串口发包，不负责视觉识别算法本身。
 * 3. 机械手执行层仍然由 MechanicalHand_MoveTo(x, y, z, time_ms) 完成。
 * 4. 当前版本的标定参数全部通过宏定义手工填写，适合固定相机、固定安装位置的场景。
 * 5. 如果相机重新安装、镜头更换、深度比例变化或机械臂基座位置变化，需要重新标定并修改这些宏。
 */

/*
 * 标定模式切换宏
 * 1. CALIB_MODE_3D:
 *    适用于深度相机 + 相机安装固定 + 目标高度有明显变化的场景。
 *    MCU 内部会用相机内参、深度比例、外参矩阵把 (u, v, depth) 转成基座坐标。
 * 2. CALIB_MODE_PLANE:
 *    适用于目标基本都在同一工作平面上抓取的场景。
 *    MCU 内部只做 u/v 到 x/y 的平面仿射映射，z 直接使用固定宏值。
 * 3. 第一版建议先用平面模式验证链路，再切换到 3D 模式。
 */
#define CALIB_MODE_PLANE                      (0U)
#define CALIB_MODE_3D                         (1U)
#define CALIB_MODE                            CALIB_MODE_PLANE

/*
 * 相机内参和深度比例
 * 1. FX/FY/CX/CY 来自相机内参标定结果。
 * 2. depth_scale 用于把相机输出的 depth 原始值换算成 mm。
 * 3. 如果相机已经直接输出 mm，则 depth_scale 可以直接写 1.0f。
 * 4. 这里默认值用于占位，必须按相机实测结果修改。
 */
#define CALIB_FX                              (588.0123f)
#define CALIB_FY                              (588.6400f)
#define CALIB_CX                              (339.3865f)
#define CALIB_CY                              (233.3830f)
#define CALIB_DEPTH_SCALE                     (1.0f)

/*
 * 深度有效范围
 * 1. 用于过滤明显异常的深度值，避免直接把无效点换算后送给机械臂。
 * 2. 单位统一使用 mm。
 * 3. 根据相机的量程，实测修改。
 */
#define CALIB_DEPTH_MIN_MM                    (30.0f)
#define CALIB_DEPTH_MAX_MM                    (1500.0f)

/*
 * 3D 标定外参
 * 1. R_cb 是相机坐标系到机械臂基座坐标系的旋转矩阵。
 * 2. t_cb 是相机坐标系原点在机械臂基座坐标系下的平移向量，单位 mm。
 * 3. 当前默认值等价于“相机坐标系和基座坐标系方向一致，且原点重合”。
 * 4. 这组值必须由标定结果填写，默认值仅用于占位。
 */
#define CALIB_R11 (0.99880090f)
#define CALIB_R12 (-0.02079327f)
#define CALIB_R13 (-0.04432148f)
#define CALIB_R21 (0.02157609f)
#define CALIB_R22 (-0.62569612f)
#define CALIB_R23 (0.77976845f)
#define CALIB_R31 (-0.04394571f)
#define CALIB_R32 (-0.77978972f)
#define CALIB_R33 (-0.62449721f)

#define CALIB_TX_MM (26.78513430f)
#define CALIB_TY_MM (-60.52442781f)
#define CALIB_TZ_MM (317.13131385f)

/*
 * 平面标定仿射参数
 * 1. 该模式下只对 (u, v) 做二维仿射映射。
 * 2. 公式为：
 *    x_base = A11 * u + A12 * v + A13
 *    y_base = A21 * u + A22 * v + A23
 * 3. z_base 直接使用固定高度宏 CALIB_PLANE_Z_FIXED_MM。
 * 4. 默认值仅用于占位，必须根据采点结果填写。
 */
#define CALIB_A11                             (0.47288834f)
#define CALIB_A12                             (0.07878448f)
#define CALIB_A13                             (-192.33411930f)
#define CALIB_A21                             (0.04584994f)
#define CALIB_A22                             (-0.14525806f)
#define CALIB_A23                             (279.60616553f)
#define CALIB_PLANE_Z_FIXED_MM                (77.0f)

/*
 * 手工补偿参数
 * 1. MANUAL_OFFSET_* 用于整体坐标的小范围人工微调。
 *    常见用途：标定完成后仍有稳定的整体偏差，例如总是向前差 5 mm。
 * 2. TCP_OFFSET_* 是末端工具补偿的简化版本。
 *    第一版把它当成基座坐标系下的固定偏移，用于修正夹爪中心点和模型参考点之间的差。
 * 3. 如果后续夹爪姿态变化很大、需要严格工具坐标补偿，则应把 TCP 偏移和末端姿态联动计算。
 */
#define CALIB_MANUAL_OFFSET_X_MM              (0.0f)
#define CALIB_MANUAL_OFFSET_Y_MM              (0.0f)
#define CALIB_MANUAL_OFFSET_Z_MM              (0.0f)

#define CALIB_TCP_OFFSET_X_MM                 (0.0f)
#define CALIB_TCP_OFFSET_Y_MM                 (-20.0f)
#define CALIB_TCP_OFFSET_Z_MM                 (0.0f)

/*
 * 标定模块返回值
 * 1. CALIB_OK: 换算成功。
 * 2. CALIB_ERR_PARAM: 输入指针为空等参数错误。
 * 3. CALIB_ERR_DEPTH: depth 无效或超出设定范围。
 * 4. CALIB_ERR_MODE: 标定模式宏配置错误。
 */
typedef enum
{
    CALIB_OK = 0,
    CALIB_ERR_PARAM = -1,
    CALIB_ERR_DEPTH = -2,
    CALIB_ERR_MODE = -3
} calib_status_t;

/*
 * 图像点 + 深度输入结构体
 * 1. u / v 是图像坐标，单位是像素。
 * 2. depth 是深度原始值，单位取决于 depth_scale 的定义。
 * 3. 如果相机直接输出的是 mm，那么 depth_scale 建议写 1.0f。
 */
typedef struct
{
    float u;
    float v;
    float depth;
} calib_image_point_t;

/*
 * 机械臂基座坐标输出结构体
 * 1. x / y / z 全部使用机械臂基座坐标系，单位 mm。
 * 2. 该结构体的输出结果可以直接送入 MechanicalHand_MoveTo()。
 */
typedef struct
{
    float x;
    float y;
    float z;
} calib_base_point_t;

/*
 * 函数说明
 * 名称：Calibration_ImageToBase
 * 作用：
 * 1. 输入相机返回的 (u, v, depth)。
 * 2. 根据当前标定模式宏，自动执行 3D 或平面换算。
 * 3. 输出机械臂基座坐标系下的 (x_base, y_base, z_base)。
 *
 * 调用链：
 * 视觉数据接收 -> Calibration_ImageToBase() -> MechanicalHand_MoveTo()
 *
 * 使用示例：
 * Calibration_ImageToBase(u, v, depth, &x_base, &y_base, &z_base);
 * MechanicalHand_MoveTo(x_base, y_base, z_base, 1000);
 */
int Calibration_ImageToBase(float u, float v, float depth,
                            float *x_base, float *y_base, float *z_base);

/*
 * 函数说明
 * 名称：Calibration_ImageToBaseStruct
 * 作用：通过结构体传入相机坐标并接收机械臂基座坐标。
 */
int Calibration_ImageToBaseStruct(const calib_image_point_t *image_point,
                                  calib_base_point_t *base_point);

#endif
