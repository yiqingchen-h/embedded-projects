#include "encoder_motion.h"
#include "encoder_six_step.h"
#include <assert.h>
#include <stdio.h>

/* 验证真实90秒轨迹及多次DWT回绕，不使用控制器估算速度。 */
static encoder_motion_t motion;
static unsigned checks;
#define CHECK(x) do { ++checks; assert(x); } while(0)

/** @brief 拼接测量记录的64位真实累计时间。 @param p 原始记录。 @return tick。 */
static uint64_t
time_of(const encoder_motion_sample_t *p)
{ return (uint64_t)p->time_hi<<32|p->time_lo; }

/** @brief 拼接有符号机械原码位置。 @param p 原始记录。 @return 展开位置。 */
static int64_t
position_of(const encoder_motion_sample_t *p)
{ return (int64_t)((uint64_t)p->position_hi<<32|p->position_lo); }

/** @brief 两种安装方向、跨机械零点/时间回绕及批读覆盖。 @param direction 编码器安装方向。 */
static void
test_long_motion(int direction)
{
    encoder_motion_init(&motion,direction,170000000U);
    uint32_t next=0;
    int64_t initial=(int64_t)(ENCODER_SIX_TURN-100U)*direction;
    for(uint32_t ms=0;ms<=90000U;++ms)
    {
        int64_t physical=ENCODER_SIX_TURN-100U+(int64_t)direction*ms*ENCODER_SIX_TURN/60000U;
        uint32_t raw=(uint32_t)physical&(ENCODER_SIX_TURN-1U);
        encoder_motion_update(&motion,raw,(uint32_t)(UINT32_MAX-100000U+(uint64_t)ms*170000U));
        if(ms%100U==0U)
        {
            encoder_motion_sample_t rows[ENCODER_MOTION_BATCH];
            uint32_t count;
            do
            {
                uint32_t wanted=next;
                count=encoder_motion_read(&motion,&next,rows);
                CHECK(next==wanted);
                for(uint32_t j=0;j<count;++j)
                {
                    uint32_t point_ms=(next+j)*20U;
                    CHECK(time_of(&rows[j])==(uint64_t)point_ms*170000U);
                    CHECK(position_of(&rows[j])-initial==(int64_t)point_ms*ENCODER_SIX_TURN/60000U);
                    CHECK(rows[j].raw<ENCODER_SIX_TURN);
                    CHECK(rows[j].min_delta<=rows[j].max_delta);
                }
                next+=count;
            } while(count!=0U);
        }
    }
    CHECK(motion.total==4501U && next==4501U && motion.gaps==0U);
    CHECK(motion.elapsed==15300000000ULL);
    encoder_motion_sample_t rows[ENCODER_MOTION_BATCH];
    next=0U;
    CHECK(encoder_motion_read(&motion,&next,rows)==ENCODER_MOTION_BATCH);
    CHECK(next==4501U-ENCODER_MOTION_CAPACITY);
    next=motion.total+1U;
    CHECK(encoder_motion_read(&motion,&next,rows)==0U);
}

/** @brief 时间缺样如实标记，重复反馈不生成虚假点；初始化清除上一轮进度。 */
static void
test_missing_time(void)
{
    encoder_motion_init(&motion,1,170000000U);
    encoder_motion_update(&motion,100U,55U);
    encoder_motion_update(&motion,100U,55U);
    CHECK(motion.total==1U && motion.elapsed==0U);
    encoder_motion_update(&motion,101U,55U+510000U);
    CHECK(motion.gaps==1U && motion.elapsed==510000U && motion.position==101);
    encoder_motion_init(&motion,-1,170000000U);
    CHECK(motion.total==0U && !motion.initialized && motion.gaps==0U);
}

/** @brief 执行真实测量模块回归。 @return 全部通过返回0。 */
int main(void)
{
    test_long_motion(1);
    test_long_motion(-1);
    test_missing_time();
    printf("encoder mechanical motion: %u checks passed\n",checks);
    return 0;
}
