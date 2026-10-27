#include "menu.h"
#include "pic.h"

MenuState_t g_state = MENU_MAIN;
// 初始化为一个不存在的状态，确保第一次上电能进入全屏刷新
static MenuState_t g_last_state = (MenuState_t)-1;
int8_t g_cursor = 0;
int8_t g_pid_item = 0;
bool g_need_refresh = true;
PID_Data_t menu_pid_data;
Nav_PID_Data_t menu_nav_pid_data;       // 导航PID
Nav_Line_PID_Data_t menu_line_pid_data; // 循迹PID
static Speed_Data_t menu_speed;
static Mech_Debug_Data_t menu_mech_data;

// 导航菜单 EEPROM 地址分配：
// 0x40: 避障 PID
// 0x60: 循迹 PID
#define NAV_AVOID_PID_EEPROM_ADDR 0x40U
#define NAV_LINE_PID_EEPROM_ADDR 0x60U
#define NAV_AVOID_PID_MAGIC 0xEEAU
#define NAV_LINE_PID_MAGIC 0xEEBU

// ==========================================
// 菜单模块运行说明
// 1. g_state 表示当前菜单状态机所处的页面。
// 2. g_last_state 用于判断是否发生了页面切换。
//    只要页面发生切换，就执行一次整屏清空，避免旧页面残留。
// 3. g_need_refresh 是菜单刷新总开关。
//    普通页面按事件刷新；监视类页面会主动再次置位以形成持续刷新。
// 4. menu_mech_data 是机械手调试页面的运行时参数缓存。
// ==========================================

// EEPROM 保存函数
void Save_PID_To_EEPROM(void)
{
    menu_pid_data.kp = g_left_speed.pid.Kp;
    menu_pid_data.ki = g_left_speed.pid.Ki;
    menu_pid_data.kd = g_left_speed.pid.Kd;
    menu_pid_data.magic = 0xFFA;

    ptEepromDev->Write(ptEepromDev, 0x00, (unsigned char *)&menu_pid_data, sizeof(PID_Data_t));
    printf("Saved!\n");
}

// EEPROM 读取函数
void Load_PID_From_EEPROM(void)
{
    ptEepromDev->Read(ptEepromDev, 0x00, (unsigned char *)&menu_pid_data, sizeof(PID_Data_t));

    if (menu_pid_data.magic == 0xFFA)
    {
        PID_coefficient_set(menu_pid_data.kp, menu_pid_data.kd, menu_pid_data.ki);
        printf("Loaded: P=%.4f, I=%.4f, D=%.4f\n", menu_pid_data.kp, menu_pid_data.ki, menu_pid_data.kd);
    }
    else
    {
        PID_coefficient_set(1.85, 0.15, 0.0055); // 默认值
        Save_PID_To_EEPROM();
        printf("Init Defaults\n");
    }
}

// ==========================================
// 导航 PID EEPROM 读写 (地址 0x40，避开电机PID的区域)
// ==========================================
void Save_Nav_PID_To_EEPROM(void)
{
    // 先保存避障 PID
    menu_nav_pid_data.nav_kp = g_pid_avoid.Kp;
    menu_nav_pid_data.nav_ki = g_pid_avoid.Ki;
    menu_nav_pid_data.nav_kd = g_pid_avoid.Kd;
    menu_nav_pid_data.magic = NAV_AVOID_PID_MAGIC; // 独立的校验码

    ptEepromDev->Write(ptEepromDev, NAV_AVOID_PID_EEPROM_ADDR, (unsigned char *)&menu_nav_pid_data, sizeof(Nav_PID_Data_t));

    // 再保存循迹 PID（与避障 PID 独立存储，避免互相覆盖）
    menu_line_pid_data.line_kp = g_pid_line.Kp;
    menu_line_pid_data.line_ki = g_pid_line.Ki;
    menu_line_pid_data.line_kd = g_pid_line.Kd;
    menu_line_pid_data.magic = NAV_LINE_PID_MAGIC;

    ptEepromDev->Write(ptEepromDev, NAV_LINE_PID_EEPROM_ADDR, (unsigned char *)&menu_line_pid_data, sizeof(Nav_Line_PID_Data_t));
    printf("Nav PID Saved! Avoid + Line\n");
}

void Load_Nav_PID_From_EEPROM(void)
{
    bool need_save_defaults = false;

    // 先加载避障 PID
    ptEepromDev->Read(ptEepromDev, NAV_AVOID_PID_EEPROM_ADDR, (unsigned char *)&menu_nav_pid_data, sizeof(Nav_PID_Data_t));

    if (menu_nav_pid_data.magic == NAV_AVOID_PID_MAGIC)
    {
        g_pid_avoid.Kp = (float)menu_nav_pid_data.nav_kp;
        g_pid_avoid.Ki = (float)menu_nav_pid_data.nav_ki;
        g_pid_avoid.Kd = (float)menu_nav_pid_data.nav_kd;
        printf("Nav PID Loaded: P=%.4f, I=%.4f, D=%.4f\n", g_pid_avoid.Kp, g_pid_avoid.Ki, g_pid_avoid.Kd);
    }
    else
    {
        // 导航 PID 默认值
        g_pid_avoid.Kp = 1.5f;
        g_pid_avoid.Ki = 0.0f;
        g_pid_avoid.Kd = 0.5f;
        need_save_defaults = true;
        printf("Nav Avoid PID Init Defaults\n");
    }

    // 再加载循迹 PID
    ptEepromDev->Read(ptEepromDev, NAV_LINE_PID_EEPROM_ADDR, (unsigned char *)&menu_line_pid_data, sizeof(Nav_Line_PID_Data_t));
    if (menu_line_pid_data.magic == NAV_LINE_PID_MAGIC)
    {
        g_pid_line.Kp = (float)menu_line_pid_data.line_kp;
        g_pid_line.Ki = (float)menu_line_pid_data.line_ki;
        g_pid_line.Kd = (float)menu_line_pid_data.line_kd;
        printf("Line PID Loaded: P=%.4f, I=%.4f, D=%.4f\n", g_pid_line.Kp, g_pid_line.Ki, g_pid_line.Kd);
    }
    else
    {
        // 循迹 PID 默认值，和 Navigation_Init() 保持一致
        g_pid_line.Kp = 0.85f;
        g_pid_line.Ki = 0.002f;
        g_pid_line.Kd = 0.1f;
        need_save_defaults = true;
        printf("Line PID Init Defaults\n");
    }

    // 只要其中任意一套是默认初始化，就统一回写，保证下次上电直接命中校验码
    if (need_save_defaults)
    {
        Save_Nav_PID_To_EEPROM();
    }
}

// 电压更新
uint16_t voltage_data[4] = {0};
float voltage = 0.0f;
static bool voltage_up_data(void)
{
    if (get_voltage_mark == true)
    {
        ADC_Read(voltage_data, 4);
        voltage = 0.0f;
        for (unsigned char i_volatge = 0; i_volatge < 4; i_volatge++)
        {
            voltage += (float)voltage_data[i_volatge];
        }
        voltage = (float)voltage / 4.0f * 3.3f / 4096.0f * 11.0f; // 电压由1 / 11分压得
        get_voltage_mark = false;
        return true;
    }
    return false;
}

// 初始化机械手调试界面的默认参数。
// 机械手真正的初始化由 hal_entry() 负责，这里只准备调试参数。
// 当前分两类调试：
// 1. 坐标调试：x / y / z / time_ms
// 2. 单舵机调试：servo_index / servo_pwm
static void Menu_Mech_ResetData(void)
{
    menu_mech_data.x = MECH_HAND_HOME_X;
    menu_mech_data.y = MECH_HAND_HOME_Y;
    menu_mech_data.z = MECH_HAND_HOME_Z;
    menu_mech_data.time_ms = 1000U;
    menu_mech_data.servo_index = 0U;
    menu_mech_data.servo_pwm = 1500U;
    menu_mech_data.elect = 0;
    menu_mech_data.last_result = MECH_HAND_OK;
    menu_mech_data.gripper_opened = false;
    MechanicalHand_GetServoPositions(&menu_mech_data.servo_positions);
    menu_mech_data.servo_pwm = menu_mech_data.servo_positions.servo_pwm[menu_mech_data.servo_index];
    menu_mech_data.last_clamped = false;
}

// 对调试输入值做简单限幅，避免误操作输入过大。
// 说明：
// 1. x/y/z 的范围只是菜单输入范围，不代表机械臂一定可达。
// 2. servo_index 限定在 0~5，对应当前机械手使用的 6 路舵机通道。
// 3. servo_pwm 按协议底层允许范围限定为 500~2500。
static void Menu_Mech_ClampData(void)
{
    if (menu_mech_data.x > MECH_HAND_MENU_X_MAX)
        menu_mech_data.x = MECH_HAND_MENU_X_MAX;
    else if (menu_mech_data.x < MECH_HAND_MENU_X_MIN)
        menu_mech_data.x = MECH_HAND_MENU_X_MIN;

    if (menu_mech_data.y > MECH_HAND_MENU_Y_MAX)
        menu_mech_data.y = MECH_HAND_MENU_Y_MAX;
    else if (menu_mech_data.y < MECH_HAND_MENU_Y_MIN)
        menu_mech_data.y = MECH_HAND_MENU_Y_MIN;

    if (menu_mech_data.z > MECH_HAND_MENU_Z_MAX)
        menu_mech_data.z = MECH_HAND_MENU_Z_MAX;
    else if (menu_mech_data.z < MECH_HAND_MENU_Z_MIN)
        menu_mech_data.z = MECH_HAND_MENU_Z_MIN;

    if (menu_mech_data.time_ms > 9999U)
        menu_mech_data.time_ms = 9999U;

    if (menu_mech_data.servo_index > 5U)
        menu_mech_data.servo_index = 5U;

    if (menu_mech_data.servo_pwm < 500U)
        menu_mech_data.servo_pwm = 500U;
    else if (menu_mech_data.servo_pwm > 2500U)
        menu_mech_data.servo_pwm = 2500U;
}

static void Menu_Mech_SyncRuntimeState(void)
{
    MechanicalHand_GetServoPositions(&menu_mech_data.servo_positions);
    menu_mech_data.last_clamped = MechanicalHand_WasLastCommandClamped();
}

// 发送最小原始协议，直接验证 UART4 与舵机控制板链路是否有效。
// 该命令不经过逆运动学，适合排查“返回成功但机械手不动作”的问题。
static void Menu_Mech_SendRawNeutral(void)
{
    menu_mech_data.last_result = MechanicalHand_SendRawDebug(
        "{#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!}");
}

// 使用最简单的单舵机命令测试夹爪开合。
// 如果这个命令都没有动作，问题通常不在逆运动学，而在 UART4 配置、接线或协议识别。
static void Menu_Mech_ToggleGripRaw(void)
{
    /*
     * 说明：
     * 1. 保留原函数名，避免影响现有调用点，也方便你后续切回总线舵机版本。
     * 2. 当前夹爪已经切到统一夹爪接口，这里不再直接发送 raw 串口命令。
     * 3. 机械手调试页的 KEY4 与自动抓取流程共用 MechanicalHand_GripperOpen/Close()。
     */
    if (menu_mech_data.gripper_opened)
    {
        menu_mech_data.last_result = MechanicalHand_GripperClose(menu_mech_data.time_ms);
        if (MECH_HAND_OK == menu_mech_data.last_result)
        {
            menu_mech_data.gripper_opened = false;
        }
    }
    else
    {
        menu_mech_data.last_result = MechanicalHand_GripperOpen(menu_mech_data.time_ms);
        if (MECH_HAND_OK == menu_mech_data.last_result)
        {
            menu_mech_data.gripper_opened = true;
        }
    }
}
// ==========================================
// ==========================================
// 机械手菜单绘制说明
// 1. 上半区：坐标调试参数与单舵机调试参数
// 2. 中间区：S0~S5 当前软件缓存位置
// 3. 下方：SAFE 和 RAW 调试状态
// 4. 机械手页保持持续刷新，便于实时观察 S0~S5 的变化
// ==========================================
// 优化后的菜单绘制函数
// ==========================================
void Menu_Draw(void)
{
    if (!g_need_refresh)
        return;
    g_need_refresh = false;

    // 检测是否切换了界面。
    // 进入新界面时，需要重新绘制静态内容并清掉旧界面残影。
    bool is_new_screen = (g_state != g_last_state);

    // 1. 如果切换了界面，执行一次全屏清空，并更新状态记录。
    // 2. 后续由各个界面自己决定哪些内容只绘制一次，哪些内容实时刷新。
    if (is_new_screen)
    {
        LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
        g_last_state = g_state;
    }

    // 2. 根据状态绘制内容。
    if (g_state == MENU_MAIN)
    {
        // --- 静态内容 (仅在界面刚进入时绘制一次) ---
        if (is_new_screen)
        {
            LCD_ShowString(30, 10, (const uint8_t *)"== MAIN MENU ==", BLACK, WHITE, 16, 0);
        }

        // --- 动态内容 (每次按键都刷新) ---
        // 通过直接用背景色重绘字符串来覆盖旧内容，代替整屏擦除。
        if (g_cursor == 0)
            LCD_ShowString(20, 50, (const uint8_t *)"-> 1. PID Set", WHITE, BLUE, 16, 0);
        else
            LCD_ShowString(20, 50, (const uint8_t *)"   1. PID Set", BLACK, WHITE, 16, 0);

        if (g_cursor == 1)
            LCD_ShowString(20, 80, (const uint8_t *)"-> 2. Monitor", WHITE, BLUE, 16, 0);
        else
            LCD_ShowString(20, 80, (const uint8_t *)"   2. Monitor", BLACK, WHITE, 16, 0);

        if (g_cursor == 2)
            LCD_ShowString(20, 110, (const uint8_t *)"-> 3. SPEED Set", WHITE, BLUE, 16, 0);
        else
            LCD_ShowString(20, 110, (const uint8_t *)"   3. SPEED Set", BLACK, WHITE, 16, 0);

        if (g_cursor == 3)
            LCD_ShowString(20, 140, (const uint8_t *)"-> 4. Nav Set", WHITE, BLUE, 16, 0);
        else
            LCD_ShowString(20, 140, (const uint8_t *)"   4. Nav Set", BLACK, WHITE, 16, 0);

        if (g_cursor == 4)
            LCD_ShowString(20, 170, (const uint8_t *)"-> 5. Mechanical", WHITE, BLUE, 16, 0);
        else
            LCD_ShowString(20, 170, (const uint8_t *)"   5. Mechanical", BLACK, WHITE, 16, 0);
    }
    else if (g_state == MENU_PID_EDIT)
    {
        // --- 静态内容 ---
        if (is_new_screen)
        {
            LCD_ShowString(30, 10, (const uint8_t *)"== PID EDIT ==", BLACK, WHITE, 16, 0);
            LCD_ShowString(10, 140, (const uint8_t *)"Long Press 1 OK", BLUE, WHITE, 12, 0);
        }

        // --- 动态内容 (箭头位置、选中高亮、数值变化) ---
        LCD_ShowString(10, 40, (const uint8_t *)((g_pid_item == 0) ? "-> Kp:" : "   Kp:"),
                       (g_pid_item == 0) ? WHITE : BLACK, (g_pid_item == 0) ? BLUE : WHITE, 16, 0);
        LCD_ShowFloatNumSigned((float)menu_pid_data.kp, 70, 40, 2, 4,
                               (g_pid_item == 0) ? BLUE : RED, WHITE, 16);

        LCD_ShowString(10, 70, (const uint8_t *)((g_pid_item == 1) ? "-> Ki:" : "   Ki:"),
                       (g_pid_item == 1) ? WHITE : BLACK, (g_pid_item == 1) ? BLUE : WHITE, 16, 0);
        LCD_ShowFloatNumSigned((float)menu_pid_data.ki, 70, 70, 2, 4,
                               (g_pid_item == 1) ? BLUE : RED, WHITE, 16);

        LCD_ShowString(10, 100, (const uint8_t *)((g_pid_item == 2) ? "-> Kd:" : "   Kd:"),
                       (g_pid_item == 2) ? WHITE : BLACK, (g_pid_item == 2) ? BLUE : WHITE, 16, 0);
        LCD_ShowFloatNumSigned((float)menu_pid_data.kd, 70, 100, 2, 4,
                               (g_pid_item == 2) ? BLUE : RED, WHITE, 16);
    }
    else if (g_state == MENU_RUNNING)
    {
        // --- 静态内容 ---
        if (is_new_screen)
        {
            LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
            LCD_ShowString(30, 10, (const uint8_t *)"== RUNNING_PARAMETER==", WHITE, BLACK, 16, 0);
            LCD_ShowString(60, 40, (const uint8_t *)"Voltage:", WHITE, BLACK, 16, 0);
            LCD_ShowString(26, 70, (const uint8_t *)"RPM_L:", WHITE, BLACK, 16, 0);
            LCD_ShowString(138, 70, (const uint8_t *)"RPM_R:", WHITE, BLACK, 16, 0);
        }

        LCD_ShowFloatNumSigned(g_left_encoder.current_rpm, 74, 70, 4, 1, RED, BLACK, 16);
        LCD_ShowFloatNumSigned(g_right_encoder.current_rpm, 188, 70, 4, 1, RED, BLACK, 16);
        if (voltage_up_data())
        {
            LCD_ShowFloatNumSigned(voltage, 132, 40, 2, 3, RED, BLACK, 16);
        }
        g_need_refresh = true;
    }
    else if (g_state == MENU_SPEDD_SET)
    {
        // --- 静态内容 ---
        if (is_new_screen)
        {
            LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
            LCD_ShowString(30, 10, (const uint8_t *)"== MENU_SPEDD_SET ==", WHITE, BLACK, 16, 0);
            LCD_ShowString(60, 40, (const uint8_t *)"Voltage:", WHITE, BLACK, 16, 0);
            LCD_ShowString(26, 70, (const uint8_t *)"RPM_L:", WHITE, BLACK, 16, 0);
            LCD_ShowString(138, 70, (const uint8_t *)"RPM_R:", WHITE, BLACK, 16, 0);
        }

        LCD_ShowFloatNumSigned(g_left_encoder.current_rpm, 74, 70, 4, 1, RED, BLACK, 16);
        LCD_ShowFloatNumSigned(g_right_encoder.current_rpm, 188, 70, 4, 1, RED, BLACK, 16);

        LCD_ShowString(2, 100, (const uint8_t *)((menu_speed.elect == 0) ? "-> NOW_L:" : "   NOW_L:"), WHITE, BLACK, 16, 0);
        LCD_ShowString(114, 100, (const uint8_t *)((menu_speed.elect == 1) ? "-> NOW_R:" : "   NOW_R:"), WHITE, BLACK, 16, 0);
        LCD_ShowString(60, 130, (const uint8_t *)((menu_speed.elect == 2) ? "-> Angle:" : "   Angle:"), WHITE, BLACK, 16, 0);

        LCD_ShowIntNumSigned(menu_speed.L_speed, 74, 100, 4, RED, BLACK, 16);
        LCD_ShowIntNumSigned(menu_speed.R_speed, 188, 100, 4, RED, BLACK, 16);
        LCD_ShowIntNumSigned(menu_speed.angle, 132, 130, 3, RED, BLACK, 16);
        if (voltage_up_data())
        {
            LCD_ShowFloatNumSigned(voltage, 124, 40, 2, 3, RED, BLACK, 16);
        }
        g_need_refresh = true;
    }
    else if (g_state == MENU_NAV_EDIT)
    {
        uint8_t nav_compact;
        // 说明：当前实机存在 LCD_H 宏与实际可视高度不一致的情况，先强制紧凑布局，确保参数区可见。
        nav_compact = 0U;
        // --- 静态内容 ---
        if (is_new_screen)
        {
            LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
            LCD_ShowString(30, (nav_compact != 0U) ? 2 : 10, (const uint8_t *)"== NAV SET ==", WHITE, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
            LCD_ShowString(18, (nav_compact != 0U) ? 16 : 30, (const uint8_t *)"Avoid", CYAN, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
            LCD_ShowString(148, (nav_compact != 0U) ? 16 : 30, (const uint8_t *)"Line", CYAN, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
            if (nav_compact == 0U)
            {
                LCD_ShowString(5, 136, (const uint8_t *)"L1:Back L2:RUN/STOP", RED, BLACK, 16, 0);
            }
        }

        // 左列：避障 PID
        LCD_ShowString((nav_compact != 0U) ? 6 : 10, (nav_compact != 0U) ? 30 : 50, (const uint8_t *)((g_pid_item == 0) ? "->Kp:" : "  Kp:"), WHITE, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
        LCD_ShowFloatNumSigned(g_pid_avoid.Kp, (nav_compact != 0U) ? 40 : 55, (nav_compact != 0U) ? 30 : 50, 2, 4, (g_pid_item == 0) ? BLUE : RED, BLACK, (nav_compact != 0U) ? 12 : 16);
        LCD_ShowString((nav_compact != 0U) ? 6 : 10, (nav_compact != 0U) ? 44 : 75, (const uint8_t *)((g_pid_item == 1) ? "->Ki:" : "  Ki:"), WHITE, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
        LCD_ShowFloatNumSigned(g_pid_avoid.Ki, (nav_compact != 0U) ? 40 : 55, (nav_compact != 0U) ? 44 : 75, 2, 4, (g_pid_item == 1) ? BLUE : RED, BLACK, (nav_compact != 0U) ? 12 : 16);
        LCD_ShowString((nav_compact != 0U) ? 6 : 10, (nav_compact != 0U) ? 58 : 100, (const uint8_t *)((g_pid_item == 2) ? "->Kd:" : "  Kd:"), WHITE, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
        LCD_ShowFloatNumSigned(g_pid_avoid.Kd, (nav_compact != 0U) ? 40 : 55, (nav_compact != 0U) ? 58 : 100, 2, 4, (g_pid_item == 2) ? BLUE : RED, BLACK, (nav_compact != 0U) ? 12 : 16);

        // 右列：循迹 PID
        LCD_ShowString((nav_compact != 0U) ? 122 : 125, (nav_compact != 0U) ? 30 : 50, (const uint8_t *)((g_pid_item == 3) ? "->Kp:" : "  Kp:"), WHITE, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
        LCD_ShowFloatNumSigned(g_pid_line.Kp, (nav_compact != 0U) ? 156 : 170, (nav_compact != 0U) ? 30 : 50, 2, 4, (g_pid_item == 3) ? BLUE : RED, BLACK, (nav_compact != 0U) ? 12 : 16);
        LCD_ShowString((nav_compact != 0U) ? 122 : 125, (nav_compact != 0U) ? 44 : 75, (const uint8_t *)((g_pid_item == 4) ? "->Ki:" : "  Ki:"), WHITE, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
        LCD_ShowFloatNumSigned(g_pid_line.Ki, (nav_compact != 0U) ? 156 : 170, (nav_compact != 0U) ? 44 : 75, 2, 4, (g_pid_item == 4) ? BLUE : RED, BLACK, (nav_compact != 0U) ? 12 : 16);
        LCD_ShowString((nav_compact != 0U) ? 122 : 125, (nav_compact != 0U) ? 58 : 100, (const uint8_t *)((g_pid_item == 5) ? "->Kd:" : "  Kd:"), WHITE, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
        LCD_ShowFloatNumSigned(g_pid_line.Kd, (nav_compact != 0U) ? 156 : 170, (nav_compact != 0U) ? 58 : 100, 2, 4, (g_pid_item == 5) ? BLUE : RED, BLACK, (nav_compact != 0U) ? 12 : 16);

        // 底部速度项：保留为统一巡航速度调节
        LCD_ShowString((nav_compact != 0U) ? 6 : 10, (nav_compact != 0U) ? 72 : 120, (const uint8_t *)((g_pid_item == 6) ? "->Spd:" : "  Spd:"), WHITE, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
        LCD_ShowIntNumSigned(g_nav_target_speed, (nav_compact != 0U) ? 48 : 62, (nav_compact != 0U) ? 72 : 120, 4, (g_pid_item == 6) ? BLUE : RED, BLACK, (nav_compact != 0U) ? 12 : 16);

        if (g_car_state == CAR_STATE_RUN)
            LCD_ShowString((nav_compact != 0U) ? 186 : 170, (nav_compact != 0U) ? 72 : 120, (const uint8_t *)"RUN ", GREEN, BLACK, (nav_compact != 0U) ? 12 : 16, 0);
        else
            LCD_ShowString((nav_compact != 0U) ? 186 : 170, (nav_compact != 0U) ? 72 : 120, (const uint8_t *)"STOP", RED, BLACK, (nav_compact != 0U) ? 12 : 16, 0);

        // 导航调参信息区：用于同时观察舵机、视觉偏移、目标轮速与实测轮速。
        if (nav_compact != 0U)
        {
            // 小屏紧凑布局：保证 Srv/Off/V/Bad/Df/TL/TR/RL/RR/LP 都在可视区。
            LCD_ShowString(4, 84, (const uint8_t *)"Srv:", WHITE, BLACK, 12, 0);
            LCD_ShowFloatNumSigned(g_nav_debug_servo_angle, 26, 84, 3, 1, YELLOW, BLACK, 12);
            LCD_ShowString(76, 84, (const uint8_t *)"Off:", WHITE, BLACK, 12, 0);
            LCD_ShowFloatNumSigned(g_nav_line_offset, 98, 84, 4, 1, CYAN, BLACK, 12);
            LCD_ShowString(164, 84, (const uint8_t *)"Df:", WHITE, BLACK, 12, 0);
            LCD_ShowFloatNumSigned(g_nav_debug_diff_rpm, 182, 84, 3, 1, YELLOW, BLACK, 12);

            LCD_ShowString(4, 96, (const uint8_t *)"V:", WHITE, BLACK, 12, 0);
            LCD_ShowIntNumSigned((int)g_nav_line_valid, 16, 96, 1, GREEN, BLACK, 12);
            LCD_ShowString(34, 96, (const uint8_t *)"Bad:", WHITE, BLACK, 12, 0);
            LCD_ShowIntNumSigned((int)g_nav_badge_enable, 58, 96, 1, GREEN, BLACK, 12);
            LCD_ShowString(86, 96, (const uint8_t *)"LP:", WHITE, BLACK, 12, 0);
            LCD_ShowFloatNumSigned(g_nav_debug_line_pid_output, 104, 96, 4, 1, MAGENTA, BLACK, 12);

            LCD_ShowString(4, 108, (const uint8_t *)"TL:", WHITE, BLACK, 12, 0);
            LCD_ShowIntNumSigned(g_nav_target_left_speed, 22, 108, 4, CYAN, BLACK, 12);
            LCD_ShowString(122, 108, (const uint8_t *)"TR:", WHITE, BLACK, 12, 0);
            LCD_ShowIntNumSigned(g_nav_target_right_speed, 140, 108, 4, CYAN, BLACK, 12);

            LCD_ShowString(4, 120, (const uint8_t *)"RL:", WHITE, BLACK, 12, 0);
            LCD_ShowFloatNumSigned(g_left_encoder.current_rpm, 22, 120, 4, 1, GREEN, BLACK, 12);
            LCD_ShowString(122, 120, (const uint8_t *)"RR:", WHITE, BLACK, 12, 0);
            LCD_ShowFloatNumSigned(g_right_encoder.current_rpm, 140, 120, 4, 1, GREEN, BLACK, 12);
        }
        else
        {
            LCD_ShowString(6, 148, (const uint8_t *)"Srv:", WHITE, BLACK, 16, 0);
            LCD_ShowFloatNumSigned(g_nav_debug_servo_angle, 38, 148, 3, 1, YELLOW, BLACK, 16);
            LCD_ShowString(112, 148, (const uint8_t *)"Off:", WHITE, BLACK, 16, 0);
            LCD_ShowFloatNumSigned(g_nav_line_offset, 144, 148, 4, 1, CYAN, BLACK, 16);

            LCD_ShowString(6, 166, (const uint8_t *)"V:", WHITE, BLACK, 16, 0);
            LCD_ShowIntNumSigned((int)g_nav_line_valid, 20, 166, 1, GREEN, BLACK, 16);
            LCD_ShowString(48, 166, (const uint8_t *)"Bad:", WHITE, BLACK, 16, 0);
            LCD_ShowIntNumSigned((int)g_nav_badge_enable, 76, 166, 1, GREEN, BLACK, 16);
            LCD_ShowString(112, 166, (const uint8_t *)"Df:", WHITE, BLACK, 16, 0);
            LCD_ShowFloatNumSigned(g_nav_debug_diff_rpm, 138, 166, 4, 1, YELLOW, BLACK, 16);

            LCD_ShowString(6, 184, (const uint8_t *)"TL:", WHITE, BLACK, 16, 0);
            LCD_ShowIntNumSigned(g_nav_target_left_speed, 28, 184, 4, CYAN, BLACK, 16);
            LCD_ShowString(112, 184, (const uint8_t *)"TR:", WHITE, BLACK, 16, 0);
            LCD_ShowIntNumSigned(g_nav_target_right_speed, 134, 184, 4, CYAN, BLACK, 16);

            LCD_ShowString(6, 202, (const uint8_t *)"RL:", WHITE, BLACK, 16, 0);
            LCD_ShowFloatNumSigned(g_left_encoder.current_rpm, 28, 202, 4, 1, GREEN, BLACK, 16);
            LCD_ShowString(112, 202, (const uint8_t *)"RR:", WHITE, BLACK, 16, 0);
            LCD_ShowFloatNumSigned(g_right_encoder.current_rpm, 134, 202, 4, 1, GREEN, BLACK, 16);

            LCD_ShowString(6, 220, (const uint8_t *)"LP:", WHITE, BLACK, 16, 0);
            LCD_ShowFloatNumSigned(g_nav_debug_line_pid_output, 28, 220, 4, 1, MAGENTA, BLACK, 16);
        }

        // 导航界面需要持续刷新状态与参数显示。
        g_need_refresh = true;
    }
    else if (g_state == MENU_MECH_DEBUG)
    {
        // 机械手页先同步一次运行时状态。
        // 这样界面上的 S0~S5、SAFE 和 RAW 才能反映最近一次动作结果。
        Menu_Mech_SyncRuntimeState();

        // --- 静态内容 ---
        if (is_new_screen)
        {
            LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
            LCD_ShowString(18, 10, (const uint8_t *)"== MECH DEBUG ==", WHITE, BLACK, 16, 0);
            LCD_ShowString(8, 250, (const uint8_t *)"L1:Move L2:Back", GREEN, BLACK, 12, 0);
            LCD_ShowString(8, 264, (const uint8_t *)"S3:Raw/Servo S4:Grip", GREEN, BLACK, 12, 0);
        }

        // --- 动态内容 ---
        // 上半区显示末端坐标调试与单舵机调试参数。
        LCD_ShowString(8, 42, (const uint8_t *)((menu_mech_data.elect == 0) ? "->X:" : "  X:"), WHITE, BLACK, 16, 0);
        LCD_ShowFloatNumSigned(menu_mech_data.x, 52, 42, 4, 1, (menu_mech_data.elect == 0) ? BLUE : RED, BLACK, 16);

        LCD_ShowString(122, 42, (const uint8_t *)((menu_mech_data.elect == 1) ? "->Y:" : "  Y:"), WHITE, BLACK, 16, 0);
        LCD_ShowFloatNumSigned(menu_mech_data.y, 166, 42, 4, 1, (menu_mech_data.elect == 1) ? BLUE : RED, BLACK, 16);

        LCD_ShowString(8, 68, (const uint8_t *)((menu_mech_data.elect == 2) ? "->Z:" : "  Z:"), WHITE, BLACK, 16, 0);
        LCD_ShowFloatNumSigned(menu_mech_data.z, 52, 68, 4, 1, (menu_mech_data.elect == 2) ? BLUE : RED, BLACK, 16);

        LCD_ShowString(122, 68, (const uint8_t *)((menu_mech_data.elect == 3) ? "->T:" : "  T:"), WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.time_ms, 166, 68, 4, (menu_mech_data.elect == 3) ? BLUE : RED, BLACK, 16);

        // IDX / PWM 是新增的单舵机调试入口：
        // 1. IDX 选择要单独控制的舵机编号
        // 2. PWM 设置该舵机的目标位置
        // 3. 短按 KEY3 会把这组参数通过 MechanicalHand_SendSingleServo() 发出去
        LCD_ShowString(8, 94, (const uint8_t *)((menu_mech_data.elect == 4) ? "->IDX:" : "  IDX:"), WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.servo_index, 60, 94, 1, (menu_mech_data.elect == 4) ? BLUE : RED, BLACK, 16);

        LCD_ShowString(122, 94, (const uint8_t *)((menu_mech_data.elect == 5) ? "->PWM:" : "  PWM:"), WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.servo_pwm, 174, 94, 4, (menu_mech_data.elect == 5) ? BLUE : RED, BLACK, 16);

        LCD_ShowString(8, 110, (const uint8_t *)"Grip:", WHITE, BLACK, 16, 0);
        if (menu_mech_data.gripper_opened)
            LCD_ShowString(52, 110, (const uint8_t *)"OPEN ", GREEN, BLACK, 16, 0);
        else
            LCD_ShowString(52, 110, (const uint8_t *)"CLOSE", RED, BLACK, 16, 0);

        LCD_ShowString(122, 110, (const uint8_t *)"RET:", WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.last_result, 166, 110, 4, YELLOW, BLACK, 16);

        // 中间区显示六路舵机的软件缓存位置。
        // 红色表示当前值已经逼近或触碰对应通道的软件限位边界。
        LCD_ShowString(8, 136, (const uint8_t *)"S0:", WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.servo_positions.servo_pwm[0], 44, 136, 4,
                             (menu_mech_data.servo_positions.servo_pwm[0] <= MECH_HAND_SERVO0_PWM_MIN || menu_mech_data.servo_positions.servo_pwm[0] >= MECH_HAND_SERVO0_PWM_MAX) ? RED : CYAN,
                             BLACK, 16);
        LCD_ShowString(122, 136, (const uint8_t *)"S3:", WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.servo_positions.servo_pwm[3], 158, 136, 4,
                             (menu_mech_data.servo_positions.servo_pwm[3] <= MECH_HAND_SERVO3_PWM_MIN || menu_mech_data.servo_positions.servo_pwm[3] >= MECH_HAND_SERVO3_PWM_MAX) ? RED : CYAN,
                             BLACK, 16);

        LCD_ShowString(8, 162, (const uint8_t *)"S1:", WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.servo_positions.servo_pwm[1], 44, 162, 4,
                             (menu_mech_data.servo_positions.servo_pwm[1] <= MECH_HAND_SERVO1_PWM_MIN || menu_mech_data.servo_positions.servo_pwm[1] >= MECH_HAND_SERVO1_PWM_MAX) ? RED : CYAN,
                             BLACK, 16);
        LCD_ShowString(122, 162, (const uint8_t *)"S4:", WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.servo_positions.servo_pwm[4], 158, 162, 4,
                             (menu_mech_data.servo_positions.servo_pwm[4] <= MECH_HAND_SERVO4_PWM_MIN || menu_mech_data.servo_positions.servo_pwm[4] >= MECH_HAND_SERVO4_PWM_MAX) ? RED : CYAN,
                             BLACK, 16);

        LCD_ShowString(8, 188, (const uint8_t *)"S2:", WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.servo_positions.servo_pwm[2], 44, 188, 4,
                             (menu_mech_data.servo_positions.servo_pwm[2] <= MECH_HAND_SERVO2_PWM_MIN || menu_mech_data.servo_positions.servo_pwm[2] >= MECH_HAND_SERVO2_PWM_MAX) ? RED : CYAN,
                             BLACK, 16);
        LCD_ShowString(122, 188, (const uint8_t *)"S5:", WHITE, BLACK, 16, 0);
        LCD_ShowIntNumSigned(menu_mech_data.servo_positions.servo_pwm[5], 158, 188, 4,
                             (menu_mech_data.servo_positions.servo_pwm[5] <= MECH_HAND_SERVO5_PWM_MIN || menu_mech_data.servo_positions.servo_pwm[5] >= MECH_HAND_SERVO5_PWM_MAX) ? RED : CYAN,
                             BLACK, 16);

        LCD_ShowString(8, 214, (const uint8_t *)"SAFE:", WHITE, BLACK, 12, 0);
        if (menu_mech_data.last_clamped)
            LCD_ShowString(50, 214, (const uint8_t *)"CLAMP", RED, BLACK, 12, 0);
        else
            LCD_ShowString(50, 214, (const uint8_t *)"OK   ", GREEN, BLACK, 12, 0);

        // RAW 显示最近一次下发给机械手模块的原始协议命令。
        // 这里只显示前 24 个字符，避免超长命令把底部布局撑乱。
        LCD_ShowString(8, 232, (const uint8_t *)"RAW:", CYAN, BLACK, 12, 0);
        {
            const char *raw_cmd = MechanicalHand_GetLastCommand();
            char raw_short[25] = {0};
            uint8_t i = 0;

            while ((raw_cmd[i] != '\0') && (i < 24U))
            {
                raw_short[i] = raw_cmd[i];
                i++;
            }
            raw_short[i] = '\0';
            LCD_ShowString(40, 232, (const uint8_t *)raw_short, YELLOW, BLACK, 12, 0);
        }
        g_need_refresh = true;
    }
}

// ==========================================
// 按键处理
// 机械手页说明：
// 1. KEY1/KEY2 短按：切换当前编辑项
// 2. 编码器旋转：修改当前编辑项数值
// 3. KEY1 长按：执行坐标运动
// 4. KEY2 长按：返回主菜单
// 5. KEY3 短按：
//    - 选中 IDX/PWM 时，执行单舵机调试
//    - 其他项时，执行原来的 4 轴 Raw4 测试
// 6. KEY4 短按：切换夹爪开合
// ==========================================
void Menu_Key_Process(key_event_t event)
{
    if (g_state == MENU_MAIN)
    {
        if (event.type == KEY_EVENT_SHORT)
        {
            // 主菜单短按：上下移动光标。
            if (event.id == KEY_ID_1)
            {
                g_cursor++;
                g_cursor = g_cursor > 4 ? 0 : g_cursor;
            }
            else if (event.id == KEY_ID_2)
            {
                g_cursor--;
                g_cursor = g_cursor < 0 ? 4 : g_cursor;
            }

            g_need_refresh = true;
        }
        else if (event.type == KEY_EVENT_LONG && event.id == KEY_ID_1)
        {
            // 主菜单长按 KEY1：进入当前选中的功能页面。
            if (g_cursor == 0)
            {
                menu_pid_data.kp = g_left_speed.pid.Kp;
                menu_pid_data.ki = g_left_speed.pid.Ki;
                menu_pid_data.kd = g_left_speed.pid.Kd;
                g_state = MENU_PID_EDIT;
            }
            if (g_cursor == 1)
                g_state = MENU_RUNNING;
            if (g_cursor == 2)
            {
                menu_speed.L_speed = 0;
                menu_speed.R_speed = 0;
                menu_speed.angle = 0;
                menu_speed.elect = 0;
                g_state = MENU_SPEDD_SET;
            }
            if (g_cursor == 3)
            {
                g_pid_item = 0;
                g_state = MENU_NAV_EDIT;
            }
            if (g_cursor == 4)
            {
                Menu_Mech_ResetData();
                g_state = MENU_MECH_DEBUG;
            }
            g_need_refresh = true;
        }
    }
    else if (g_state == MENU_PID_EDIT)
    {
        if (event.type == KEY_EVENT_SHORT)
        {
            // PID 页面短按：切换当前编辑项。
            if (event.id == KEY_ID_1)
            {
                g_pid_item++;
                if (g_pid_item > 2)
                    g_pid_item = 0;
            }
            else if (event.id == KEY_ID_2)
            {
                g_pid_item--;
                if (g_pid_item < 0)
                    g_pid_item = 2;
            }
            g_need_refresh = true;
        }
        else if (event.type == KEY_EVENT_LONG)
        {
            if (event.id == KEY_ID_1)
            {
                // 长按 KEY1：确认保存并返回主菜单。
                PID_coefficient_set(menu_pid_data.kp, menu_pid_data.kd, menu_pid_data.ki);
                Save_PID_To_EEPROM();
                g_state = MENU_MAIN;
                g_need_refresh = true;
            }
            else if (event.id == KEY_ID_2)
            {
                // 长按 KEY2：取消修改并返回。
                g_state = MENU_MAIN;
                g_need_refresh = true;
            }
        }
        else if (event.type == KEY_EVENT_ROT_CW || event.type == KEY_EVENT_ROT_CCW)
        {
            // 编码器：按当前 Kp/Ki/Kd 项分别使用不同步进调整。
            float step = 0.0f;
            if (g_pid_item == 0)
                step = 0.1f;
            else if (g_pid_item == 1)
                step = 0.01f;
            else if (g_pid_item == 2)
                step = 0.001f;

            step *= (float)event.count;
            if (g_pid_item == 0)
                menu_pid_data.kp += step;
            else if (g_pid_item == 1)
                menu_pid_data.ki += step;
            else if (g_pid_item == 2)
                menu_pid_data.kd += step;

            Key_Set_Encoder_Count(0);
            g_need_refresh = true;
        }
    }
    else if (g_state == MENU_RUNNING)
    {
        if (event.type == KEY_EVENT_LONG && event.id == KEY_ID_2)
        {
            // 监视页长按 KEY2：返回主菜单。
            g_state = MENU_MAIN;
            g_need_refresh = true;
        }
        if (event.type == KEY_EVENT_SHORT)
        {
            if (event.id == KEY_ID_4)
            {
                // 短按 KEY4：停止车辆并将转向舵机回正。
                wheel_stop();
                Servo_SetAngle(0);
                g_state = MENU_RUNNING;
            }
        }
    }
    else if (g_state == MENU_SPEDD_SET)
    {
        if (event.type == KEY_EVENT_SHORT)
        {
            // 速度页短按：切换当前编辑项，或执行停止复位。
            if (event.id == KEY_ID_1)
            {
                menu_speed.elect++;
                if (menu_speed.elect > 2)
                    menu_speed.elect = 0;
            }
            else if (event.id == KEY_ID_2)
            {
                menu_speed.elect--;
                if (menu_speed.elect < 0)
                    menu_speed.elect = 2;
            }
            else if (event.id == KEY_ID_4)
            {
                // 停止运行并复位显示参数。
                wheel_stop();
                Servo_SetAngle(0);
                menu_speed.L_speed = 0;
                menu_speed.R_speed = 0;
                menu_speed.angle = 0;
                menu_speed.elect = 0;
                g_state = MENU_SPEDD_SET;
            }
            g_need_refresh = true;
        }
        else if (event.type == KEY_EVENT_ROT_CW || event.type == KEY_EVENT_ROT_CCW)
        {
            // 编码器：按当前选中项调节左右轮速度或舵机角度。
            if (menu_speed.elect == 0)
                menu_speed.L_speed += event.count;
            else if (menu_speed.elect == 1)
                menu_speed.R_speed += event.count;
            else if (menu_speed.elect == 2)
                menu_speed.angle += event.count;

            Key_Set_Encoder_Count(0);
            g_need_refresh = true;
        }
        if (event.type == KEY_EVENT_LONG)
        {
            if (event.id == KEY_ID_1)
            {
                // 长按 KEY1：把当前调试值下发到速度控制与转向控制。
                g_state = MENU_SPEDD_SET;
                g_left_speed.pid.target_speed = menu_speed.L_speed;
                g_right_speed.pid.target_speed = menu_speed.R_speed;
                Servo_SetAngle(menu_speed.angle);
                g_need_refresh = true;
            }
            else if (event.id == KEY_ID_2)
            {
                // 长按 KEY2：返回主菜单。
                g_state = MENU_MAIN;
                g_need_refresh = true;
            }
        }
    }
    else if (g_state == MENU_NAV_EDIT)
    {
        if (event.type == KEY_EVENT_SHORT)
        {
            // 导航页短按：切换编辑项，或保存后退出。
            if (event.id == KEY_ID_1)
            {
                g_pid_item++;
                if (g_pid_item > 6)
                    g_pid_item = 0;
            }
            else if (event.id == KEY_ID_2)
            {
                g_pid_item--;
                if (g_pid_item < 0)
                    g_pid_item = 6;
            }
            else if (event.id == KEY_ID_4)
            {
                // 短按 KEY4：保存导航页两套 PID，并停止车辆。
                Save_Nav_PID_To_EEPROM();
                g_state = MENU_MAIN;
                g_car_state = CAR_STATE_STOP;
                wheel_stop();
                Servo_SetAngle(0);
            }
            g_need_refresh = true;
        }
        else if (event.type == KEY_EVENT_LONG)
        {
            if (event.id == KEY_ID_1)
            {
                // 长按 KEY1：RUN / STOP 切换。
                if (g_car_state == CAR_STATE_STOP)
                    g_car_state = CAR_STATE_RUN;
                else
                    g_car_state = CAR_STATE_STOP;
            }
            else if (event.id == KEY_ID_2)
            {
                // 长按 KEY2：取消修改，重新加载 EEPROM 中的旧值并退出。
                Load_Nav_PID_From_EEPROM();
                g_state = MENU_MAIN;
                g_car_state = CAR_STATE_STOP;
                wheel_stop();
                Servo_SetAngle(0);
            }
            g_need_refresh = true;
        }
        else if (event.type == KEY_EVENT_ROT_CW || event.type == KEY_EVENT_ROT_CCW)
        {
            // 编码器：
            // 0~2 调避障 PID；3~5 调循迹 PID；6 调统一巡航速度。
            float step = 0.0f;
            if (g_pid_item == 0 || g_pid_item == 3)
                step = 0.1f;
            else if (g_pid_item == 1 || g_pid_item == 2 || g_pid_item == 4 || g_pid_item == 5)
                step = 0.01f;

            if (g_pid_item < 6)
            {
                step *= (float)event.count;
                if (g_pid_item == 0)
                    g_pid_avoid.Kp += step;
                else if (g_pid_item == 1)
                    g_pid_avoid.Ki += step;
                else if (g_pid_item == 2)
                    g_pid_avoid.Kd += step;
                else if (g_pid_item == 3)
                    g_pid_line.Kp += step;
                else if (g_pid_item == 4)
                    g_pid_line.Ki += step;
                else if (g_pid_item == 5)
                    g_pid_line.Kd += step;
            }
            else
            {
                g_nav_target_speed += event.count;
            }

            Key_Set_Encoder_Count(0);
            g_need_refresh = true;
        }
    }
    else if (g_state == MENU_MECH_DEBUG)
    {
        if (event.type == KEY_EVENT_SHORT)
        {
            // 机械手页短按：
            // 1. KEY1 / KEY2 切换当前编辑项。
            // 2. KEY3 执行动作：
            //    - 选中 IDX / PWM 时走单舵机调试；
            //    - 其他项时保留原来的 Raw4 测试。
            // 3. KEY4 切换夹爪开合。
            if (event.id == KEY_ID_1)
            {
                menu_mech_data.elect++;
                if (menu_mech_data.elect > 5)
                    menu_mech_data.elect = 0;
            }
            else if (event.id == KEY_ID_2)
            {
                menu_mech_data.elect--;
                if (menu_mech_data.elect < 0)
                    menu_mech_data.elect = 5;
            }
            else if (event.id == KEY_ID_3)
            {
                // 单舵机调试逻辑：
                // 1. 当选中 IDX 或 PWM 时，直接发送单舵机协议。
                // 2. 其他情况下保持原来的 4 轴 Raw4 测试逻辑不变。
                if ((menu_mech_data.elect == 4) || (menu_mech_data.elect == 5))
                {
                    menu_mech_data.last_result = MechanicalHand_SendSingleServo(menu_mech_data.servo_index,
                                                                                menu_mech_data.servo_pwm,
                                                                                menu_mech_data.time_ms);
                }
                else
                {
                    Menu_Mech_SendRawNeutral();
                }
            }
            else if (event.id == KEY_ID_4)
            {
                // 当前 KEY4 统一走夹爪开合接口，便于与自动抓取流程保持一致。
                Menu_Mech_ToggleGripRaw();
            }
            g_need_refresh = true;
        }
        else if (event.type == KEY_EVENT_LONG)
        {
            if (event.id == KEY_ID_1)
            {
                // 长按 KEY1：执行末端坐标运动。
                menu_mech_data.last_result = MechanicalHand_MoveTo(menu_mech_data.x, menu_mech_data.y, menu_mech_data.z, menu_mech_data.time_ms);
            }
            else if (event.id == KEY_ID_2)
            {
                // 长按 KEY2：退出机械手调试页，返回主菜单。
                g_state = MENU_MAIN;
            }
            g_need_refresh = true;
        }
        else if (event.type == KEY_EVENT_ROT_CW || event.type == KEY_EVENT_ROT_CCW)
        {
            // 编码器：根据当前 elect 修改对应参数。
            // 0~3 为坐标调试项，4~5 为单舵机调试项。
            if (menu_mech_data.elect == 0)
                menu_mech_data.x += (float)event.count;
            else if (menu_mech_data.elect == 1)
                menu_mech_data.y += (float)event.count;
            else if (menu_mech_data.elect == 2)
                menu_mech_data.z += (float)event.count;
            else if (menu_mech_data.elect == 3)
            {
                int32_t temp_time = (int32_t)menu_mech_data.time_ms + (event.count * 50);
                if (temp_time < 0)
                    temp_time = 0;
                menu_mech_data.time_ms = (uint16_t)temp_time;
            }
            else if (menu_mech_data.elect == 4)
            {
                // 调整单舵机调试编号，范围固定为 0~5。
                // 当编号切换后，同步把 PWM 刷成该路当前缓存值，便于从当前位置继续微调。
                int32_t temp_index = (int32_t)menu_mech_data.servo_index + event.count;
                if (temp_index < 0)
                    temp_index = 0;
                else if (temp_index > 5)
                    temp_index = 5;
                menu_mech_data.servo_index = (uint8_t)temp_index;
                menu_mech_data.servo_pwm = menu_mech_data.servo_positions.servo_pwm[menu_mech_data.servo_index];
            }
            else if (menu_mech_data.elect == 5)
            {
                // 调整单舵机调试 PWM。
                // 这里不直接按某一路舵机限位截断，而是先按协议底层范围处理，
                // 最终仍由 MechanicalHand_SendSingleServo() 内部做对应通道的软件限位。
                int32_t temp_pwm = (int32_t)menu_mech_data.servo_pwm + (event.count * 10);
                if (temp_pwm < 0)
                    temp_pwm = 0;
                menu_mech_data.servo_pwm = (uint16_t)temp_pwm;
            }

            Menu_Mech_ClampData();
            Key_Set_Encoder_Count(0);
            g_need_refresh = true;
        }
    }
}
