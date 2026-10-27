#include "control_model.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned checks; /* 契约断言计数。 */
#define CHECK(x) do { ++checks; assert(x); } while(0)
/** @brief 确认一个在途操作。 @param success 后端是否成功。 @return 操作编号。 */
static uint32_t complete(bool success)
{
    control_request_t r; CHECK(control_model_take_request(&r)); CHECK(!control_model_take_request(&r));
    CHECK(!control_model_complete(r.sequence+1U,true,NULL)); CHECK(control_model_complete(r.sequence,success,success?NULL:"FAILED")); return r.sequence;
}
/** @brief 验证独立六步请求范围、复制、停机互锁和识别隔离。 */
static void
test_step_request(void)
{
    control_model_init();
    control_step_test_t test={.sector=0,.dwell_ms=20};
    CHECK(control_model_request_step(NULL,100)==CONTROL_INVALID);
    CHECK(control_model_request(CONTROL_REQUEST_STEP_TEST,CONTROL_PARAM_TARGET,0,100)==CONTROL_INVALID);
    CHECK(control_model_request((control_request_kind_t)(CONTROL_REQUEST_STEP_TEST+1U),CONTROL_PARAM_TARGET,0,100)==CONTROL_INVALID);
    CHECK(control_model_request((control_request_kind_t)-1,CONTROL_PARAM_TARGET,0,100)==CONTROL_INVALID);
    CHECK(control_model_request_step(&test,100)==CONTROL_NOT_READY);
    control_snapshot_t s={.algorithm=control_model_algorithm(),.mode=control_model_mode(),
        .ready=true,.run=CONTROL_STOPPED,.power_disabled=true,.updated_ms=100,
        .capabilities=CONTROL_CAP_STEP_TEST|CONTROL_CAP_STOP|CONTROL_CAP_START|CONTROL_CAP_IDENTIFY};
    CHECK(control_model_publish(&s));
    CHECK(control_model_request_step(&test,1101)==CONTROL_NOT_READY);
    s.capabilities&=~CONTROL_CAP_STEP_TEST; CHECK(control_model_publish(&s));
    CHECK(control_model_request_step(&test,100)==CONTROL_NOT_READY);
    s.capabilities|=CONTROL_CAP_STEP_TEST; s.power_disabled=false; CHECK(control_model_publish(&s));
    CHECK(control_model_request_step(&test,100)==CONTROL_BUSY);
    s.power_disabled=true; s.run=CONTROL_RUNNING; CHECK(control_model_publish(&s));
    CHECK(control_model_request_step(&test,100)==CONTROL_BUSY);
    s.run=CONTROL_FAULT; CHECK(control_model_publish(&s));
    CHECK(control_model_request_step(&test,100)==CONTROL_BUSY);
    s.run=CONTROL_DISABLED; CHECK(control_model_publish(&s));
    test.sector=6; CHECK(control_model_request_step(&test,100)==CONTROL_RANGE);
    test.sector=0; test.dwell_ms=19; CHECK(control_model_request_step(&test,100)==CONTROL_RANGE);
    test.dwell_ms=1001; CHECK(control_model_request_step(&test,100)==CONTROL_RANGE);
    test.dwell_ms=20; test.reverse=true; CHECK(control_model_request_step(&test,100)==CONTROL_RANGE);
    test.cycle=true; test.sector=1; CHECK(control_model_request_step(&test,100)==CONTROL_RANGE);
    test.sector=0; CHECK(control_model_request_step(&test,100)==CONTROL_OK);
    CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_BUSY);
    CHECK(control_model_request_step(&test,100)==CONTROL_BUSY);
    CHECK(control_model_request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_BUSY);
    /* STOP 可覆盖尚未取走的测试配置，不能让排队测试稍后启动。 */
    CHECK(control_model_request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK);
    CHECK(control_model_pending()->kind==CONTROL_REQUEST_STOP);
    CHECK(control_model_pending()->step_test.dwell_ms==0U); complete(true);
    CHECK(control_model_request_step(&test,100)==CONTROL_OK);
    control_request_t operation; CHECK(control_model_take_request(&operation));
    CHECK(operation.kind==CONTROL_REQUEST_STEP_TEST && operation.step_test.cycle && operation.step_test.reverse);
    CHECK(control_model_request(CONTROL_REQUEST_STOP,CONTROL_PARAM_TARGET,0,100)==CONTROL_BUSY);
    CHECK(control_model_complete(operation.sequence,true,NULL));
    control_parameter_t p={.id=CONTROL_PARAM_TARGET,.value=300,.minimum=0,.maximum=5400,
        .step=1,.valid=true,.editable=true}; strcpy(p.name,"TARGET");
    CHECK(control_model_parameter_publish(&p));
    s.run=CONTROL_STOPPED; CHECK(control_model_publish(&s));
    /* calibrated=false 是真实停机状态；通电前校准由后端执行。 */
    CHECK(!control_model_snapshot()->calibrated);
    test.cycle=false; test.reverse=false;
    for (unsigned sector=0;sector<6U;++sector)
    {
        for (unsigned edge=0;edge<2U;++edge)
        {
            test.sector=(uint8_t)sector; test.dwell_ms=edge==0U?20U:1000U;
            CHECK(control_model_request_step(&test,100)==CONTROL_OK);
            CHECK(control_model_take_request(&operation));
            CHECK(operation.step_test.sector==sector && operation.step_test.dwell_ms==test.dwell_ms);
            test.sector=0; CHECK(operation.step_test.sector==sector);
            CHECK(control_model_complete(operation.sequence,true,NULL));
            CHECK(control_model_parameter(CONTROL_PARAM_TARGET)->value==300);
        }
    }
    CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK);
    complete(true); CHECK(control_model_request_step(&test,100)==CONTROL_BUSY);
    CHECK(control_model_request(CONTROL_REQUEST_ID_CANCEL,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK); complete(true);
    CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK);
    CHECK(control_model_take_request(&operation));
    CHECK(control_model_complete(operation.sequence,false,"ID FAILURE"));
    CHECK(control_model_request_step(&test,100)==CONTROL_OK); CHECK(control_model_take_request(&operation));
    CHECK(control_model_complete(operation.sequence,false,"STEP FAILURE"));
    CHECK(strcmp(control_model_identification()->reason,"ID FAILURE")==0);
    CHECK(strcmp(control_model_notice(),"STEP FAILURE")==0);
    CHECK(control_model_request_step(&test,100)==CONTROL_OK); complete(true);
    CHECK(strcmp(control_model_identification()->reason,"ID FAILURE")==0);
}
/** @brief 模型生命周期与互锁回归。 @return 测试退出码。 */
int main(void)
{
    control_model_init(); CHECK(!control_model_snapshot()->ready);
    CHECK(control_model_request(CONTROL_REQUEST_START,CONTROL_PARAM_TARGET,0,0)==CONTROL_NOT_READY);
    for(unsigned a=0;a<CONTROL_ALGORITHM_COUNT;a++) { for(unsigned m=0;m<CONTROL_MODE_COUNT;m++) {
        bool allowed=m==CONTROL_SPEED || (m==CONTROL_TORQUE&&control_algorithm_foc((control_algorithm_t)a)) ||
            ((m==CONTROL_POSITION||m==CONTROL_CURRENT||m==CONTROL_HAPTIC)&&a==CONTROL_ENCODER_FOC);
        CHECK(control_model_select((control_algorithm_t)a,(control_mode_t)m)==(allowed?CONTROL_OK:CONTROL_INVALID));
    } }
    control_snapshot_t s={0}; s.algorithm=control_model_algorithm();s.mode=control_model_mode(); s.ready=true;s.run=CONTROL_STOPPED;s.power_disabled=true;s.capabilities=127;s.updated_ms=100;
    CHECK(control_model_publish(&s)); CHECK(!control_model_stale(1100)); CHECK(control_model_stale(1101));
    control_parameter_t p={.id=CONTROL_PARAM_POLES,.value=5,.minimum=1,.maximum=20,.step=1,.valid=true,.editable=true}; strcpy(p.name,"POLE PAIRS");
    CHECK(control_model_parameter_publish(&p));
    CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK);
    CHECK(control_model_id_locked()); CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_BUSY);
    CHECK(control_model_request(CONTROL_REQUEST_ID_CANCEL,CONTROL_PARAM_TARGET,0,100)==CONTROL_BUSY);
    complete(true);
    control_id_report_t r=*control_model_identification(); uint32_t old_task=r.task_id;
    r.stage=CONTROL_ID_ROTATING;r.turns_milli=2000;r.progress=1000;
    CHECK(control_model_id_publish(&r)); CHECK(control_model_identification()->stage==CONTROL_ID_ROTATING);
    CHECK(control_model_request(CONTROL_REQUEST_PARAMETER,CONTROL_PARAM_POLES,7,100)==CONTROL_BUSY);
    CHECK(!control_model_parameter_publish(&p));
    CHECK(control_model_request(CONTROL_REQUEST_ID_CANCEL,CONTROL_PARAM_TARGET,0,5000)==CONTROL_OK);
    CHECK(!control_model_id_publish(&r)); complete(false); CHECK(control_model_id_locked());
    CHECK(control_model_request(CONTROL_REQUEST_ID_CANCEL,CONTROL_PARAM_TARGET,0,5000)==CONTROL_OK); complete(true);
    CHECK(!control_model_id_locked() && control_model_identification()->stage==CONTROL_ID_CANCELLED);
    CHECK(!control_model_id_publish(&r)); CHECK(control_model_parameter(CONTROL_PARAM_POLES)->value==5);
    CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK); complete(true);
    CHECK(control_model_identification()->task_id!=old_task); CHECK(!control_model_id_publish(&r));
    r=*control_model_identification();r.stage=CONTROL_ID_FAILED;strcpy(r.reason,"NO ANGLE"); CHECK(control_model_id_publish(&r));CHECK(!control_model_id_locked());
    CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK); complete(true);
    r=*control_model_identification();r.stage=CONTROL_ID_SUCCESS;r.result_count=1;r.results[0]=p;r.results[0].value=7;
    CHECK(control_model_id_publish(&r));CHECK(control_model_parameter(CONTROL_PARAM_POLES)->value==5);
    CHECK(control_model_request(CONTROL_REQUEST_ID_APPLY,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK); complete(false);
    CHECK(control_model_id_locked());CHECK(control_model_parameter(CONTROL_PARAM_POLES)->value==5);
    CHECK(control_model_request(CONTROL_REQUEST_ID_APPLY,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK); complete(true);
    CHECK(control_model_parameter(CONTROL_PARAM_POLES)->value==7); CHECK(control_model_identification()->stage==CONTROL_ID_APPLIED);CHECK(control_model_identification()->reason[0]==0);
    CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK); complete(true);
    r=*control_model_identification();r.stage=CONTROL_ID_SUCCESS;r.result_count=1;r.results[0]=p;
    CHECK(control_model_id_publish(&r)); CHECK(control_model_request(CONTROL_REQUEST_ID_DISCARD,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK);complete(true);
    CHECK(control_model_parameter(CONTROL_PARAM_POLES)->value==7);
    CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_OK);complete(true);
    r=*control_model_identification();r.result_count=9;CHECK(!control_model_id_publish(&r));CHECK(control_model_identification()->stage==CONTROL_ID_FAILED);CHECK(!control_model_id_locked());
    s.run=CONTROL_RUNNING;CHECK(control_model_publish(&s));CHECK(control_model_select(CONTROL_OPEN_6STEP,CONTROL_SPEED)==CONTROL_BUSY);CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_BUSY);
    s.run=CONTROL_DISABLED;s.power_disabled=false;CHECK(control_model_publish(&s));CHECK(control_model_request(CONTROL_REQUEST_ID_START,CONTROL_PARAM_TARGET,0,100)==CONTROL_BUSY);
    s.updated_ms=UINT32_MAX-10U;CHECK(control_model_publish(&s));CHECK(!control_model_stale(20));
    char text[32];control_format(text,sizeof(text),INT32_MIN,3,"A");CHECK(strcmp(text,"-2147483.648 A")==0);
    test_step_request();
    printf("control model: %u checks passed\n",checks);return 0;
}
