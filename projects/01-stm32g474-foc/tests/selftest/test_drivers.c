#include "drv8353f.h"
#include "mt6835.h"
#include "test_logic.h"
#include <stdio.h>
#include <string.h>

/* 统计检查数、失败数。 */
static unsigned g_checks, g_failures;
/* 模拟DRV寄存器、锁定及逐事务故障注入。 */
static uint16_t g_registers[7];
static unsigned g_drv_mode, g_drv_calls, g_fail_at;
static bool g_locked;
/* 模拟MT用户影子寄存器、故障模式及非法命令检测。 */
static uint8_t g_user_id;
static unsigned g_mt_mode, g_illegal;

/** @brief 验证条件。 @param pass 条件。 @param label 名称。 */
static void
check(bool pass, const char *label)
{
    ++g_checks;
    if (!pass) { ++g_failures; printf("FAIL %s\n",label); }
}

/** @brief 模拟DRV的16位寄存器协议和锁定规则。 @param tx 请求。 @param rx 响应。 @return 传输结果。 */
static bool
drv_bus(uint16_t tx, uint16_t *rx)
{
    ++g_drv_calls;
    if (g_drv_mode == 4U || (g_fail_at != 0U && g_drv_calls == g_fail_at)) { return false; }
    if (g_drv_mode == 1U) { *rx=0xFFFFU; return true; }
    if (g_drv_mode == 2U) { *rx=0U; return true; }
    unsigned address=(tx>>11)&15U;
    if (address>6U) { return false; }
    *rx=(uint16_t)(0xA800U | g_registers[address]);
    if ((tx&0x8000U)==0U && g_drv_mode != 3U)
    {
        uint16_t value=tx&0x7FFU;
        if (address==3U)
        {
            if ((value&0x700U)==0x300U) { g_locked=false; }
            if (!g_locked) { g_registers[address]=value; }
            if ((value&0x700U)==0x600U) { g_locked=true; }
        }
        else if (!g_locked) { g_registers[address]=value; }
        else if (address==2U) { g_registers[2]=(uint16_t)((g_registers[2]&~7U)|(value&7U)); }
    }
    return true;
}

/** @brief 重置模拟DRV。 @param mode 故障模式。 */
static void
reset_drv(unsigned mode)
{
    memset(g_registers,0,sizeof(g_registers));
    g_registers[3]=0x3FFU;
    g_locked=false; g_drv_calls=0U; g_fail_at=0U; g_drv_mode=mode;
}

/** @brief 模拟MT，只允许用户ID影子寄存器和只读角度，其他命令一律失败。 @param tx 请求。 @param rx 响应。 @param count 字节数。 @return 成功为true。 */
static bool
mt_bus(const uint8_t *tx,uint8_t *rx,uint16_t count)
{
    memset(rx,0,count);
    if (g_mt_mode==4U) { return false; }
    if (g_mt_mode==1U) { memset(rx,0xFF,count); return true; }
    if (g_mt_mode==2U) { return true; }
    if (count==3U && tx[1]==1U && (tx[0]==0x30U || tx[0]==0x60U))
    {
        if (tx[0]==0x30U) { rx[2]=g_user_id; }
        else if (g_mt_mode!=3U) { g_user_id=tx[2]; }
        return true;
    }
    if (count==6U && tx[0]==0xA0U && tx[1]==3U)
    {
        rx[2]=0x12U; rx[3]=0x34U; rx[4]=0x56U;
        /* 独立已知向量：CRC-8/SMBUS(12 34 56)=7C。 */
        rx[5]=g_mt_mode==5U ? 0x7DU : 0x7CU;
        return true;
    }
    ++g_illegal;
    return false;
}

/** @brief 验证协议、假应答、每个配置事务失败及功率门禁。 @return 测试退出码。 */
int
main(void)
{
    uint16_t value, f1, f2;
    mt6835_sample_t sample;
    check(!drv8353f_init(NULL),"null DRV bus");
    reset_drv(0U);
    check(drv8353f_init(drv_bus),"DRV init and lock");
    unsigned calls=g_drv_calls;
    check(g_locked && g_registers[2]==0x484U && g_registers[3]==0x600U,"coast and lock");
    check(g_registers[4]==0x300U && g_registers[5]==0x310U && g_registers[6]==0x280U,"latched OCP and gain");
    check(drv8353f_coast(false) && drv8353f_verify(false),"coast release under lock");
    check(drv8353f_coast(true) && drv8353f_verify(true),"coast restore under lock");
    check(!drv8353f_read(7U,&value) && !drv8353f_read(2U,NULL),"invalid read");
    g_registers[0]=0x401U; g_registers[1]=0x20U;
    check(drv8353f_faults(&f1,&f2) && f1==0x401U && f2==0x20U,"fault fields");
    g_registers[5]^=0x40U;
    check(!drv8353f_verify(true),"protection corruption");
    for (unsigned mode=1U;mode<=4U;++mode)
    {
        reset_drv(mode);
        check(!drv8353f_init(drv_bus),"floating/stuck/ignored/failed DRV bus");
    }
    for (unsigned i=1U;i<=calls;++i)
    {
        reset_drv(0U); g_fail_at=i;
        check(!drv8353f_init(drv_bus),"DRV each transaction failure");
    }
    reset_drv(0U);
    check(drv8353f_init_gain(drv_bus,40U) && g_registers[6]==0x2C0U && g_locked,
        "load gain40 initialized and locked");
    check(!drv8353f_init_gain(drv_bus,10U),"unsupported load gain rejected");
    unsigned begin=g_drv_calls;
    check(drv8353f_calibrate(true) && g_registers[6]==0x2DCU && g_locked,
        "all three CSA inputs shorted under coast");
    unsigned calibration_calls=g_drv_calls-begin;
    check(drv8353f_calibrate(false) && g_registers[6]==0x2C0U && drv8353f_verify(true),
        "calibration leaves gain40 and normal inputs");
    check(drv8353f_coast(false) && !drv8353f_calibrate(true),"calibration requires coast");
    check(drv8353f_init(drv_bus) && g_registers[6]==0x280U,"normal retest restores gain20");
    for (unsigned i=1U;i<=calibration_calls;++i)
    {
        reset_drv(0U);
        check(drv8353f_init_gain(drv_bus,40U),"prepare calibration fault injection");
        g_fail_at=g_drv_calls+i;
        check(!drv8353f_calibrate(true),"each calibration transaction failure is propagated");
    }
    check(!mt6835_probe(NULL) && !mt6835_read(&sample),"null MT bus");
    g_user_id=0xC7U;
    check(mt6835_probe(mt_bus) && g_user_id==0xC7U,"MT two patterns and restore");
    const uint8_t vector[3]={0x12U,0x34U,0x56U};
    check(mt6835_crc(vector)==0x7CU,"known CRC vector");
    check(mt6835_read(&sample) && sample.angle==(0x123456U>>3) && sample.status==6U,"angle and warning separated");
    check(!mt6835_read(NULL),"null sample");
    for (unsigned mode=1U;mode<=4U;++mode)
    {
        g_mt_mode=mode; g_user_id=0xC7U;
        check(!mt6835_probe(mt_bus),"MT absent/stuck/write failure");
    }
    g_mt_mode=5U;
    check(mt6835_probe(mt_bus) && !mt6835_read(&sample),"CRC corruption rejected");
    check(g_illegal==0U,"no EEPROM/calibration command");
    check(test_voltage_mv(4095U,3300U)==42900U && test_voltage_mv(0U,3300U)==0U,"voltage scale");
    for (unsigned mask=0U;mask<32U;++mask)
    {
        bool ok=test_power_allowed((mask&1U)!=0U,(mask&2U)!=0U,(mask&4U)!=0U,(mask&8U)!=0U,(mask&16U)==0U,12000U);
        check(ok==(mask==31U),"every power prerequisite");
    }
    check(!test_power_allowed(true,true,true,true,false,9999U),"undervoltage block");
    check(!test_power_allowed(true,true,true,true,false,14001U),"overvoltage block");
    check(test_power_allowed(true,true,true,true,false,10000U) && test_power_allowed(true,true,true,true,false,14000U),"voltage boundaries");
    check(test_csa_valid(2048U,32U) && !test_csa_valid(2048U,33U),"CSA noise boundary");
    check(!test_csa_valid(0U,0U) && !test_csa_valid(4095U,0U),"CSA open/rail fault");
    check(test_high_side_valid(1000U,1000U) && !test_high_side_valid(0U,1000U) && !test_high_side_valid(1000U,0U),"phase response");
    printf("Selftest protocol/policy checks=%u failures=%u\n",g_checks,g_failures);
    return g_failures ? 1 : 0;
}
