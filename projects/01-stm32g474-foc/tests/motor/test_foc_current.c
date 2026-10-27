#include "foc_current.h"
#include "foc_voltage.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief 同拍Park电流、双方向、不同Ld/Lq及零磁链分别验证，积分不能冒充前馈。 */
static void
check_feedforward(void)
{
    for(int sign=-1;sign<=1;sign++)
    {
        foc_current_t s;uint16_t out[3];
        assert(foc_current_init(&s,10.1f,.0015f,800,.0001f,8500,425,8075));
        float omega=sign*200.0f;
        foc_current_feedforward(&s,omega,.0012f,.0018f,.00763f);
        assert(foc_current_step(&s,.1f,.2f,1,0,1,0,12,.1f,.2f,out));
        assert(s.integral_d==0 && s.integral_q==0);
        assert(fabsf(s.command_d_v+omega*.0018f*.2f)<.00001f);
        assert(fabsf(s.command_q_v-omega*(.0012f*.1f+.00763f))<.00001f);
        /* 单独保留交叉解耦，磁链0时不施加永磁反电动势项。 */
        foc_current_feedforward(&s,omega,.0012f,.0018f,0);
        assert(foc_current_step(&s,.1f,.2f,1,0,1,0,12,.1f,.2f,out));
        assert(fabsf(s.command_q_v-omega*.0012f*.1f)<.00001f);
        foc_current_feedforward(&s,0,.0012f,.0018f,.00763f);
        assert(foc_current_step(&s,.1f,.2f,1,0,1,0,12,.1f,.2f,out));
        assert(out[0]==4250 && out[1]==4250 && out[2]==4250);
    }
    foc_current_t s;uint16_t out[3];
    assert(foc_current_init(&s,10.1f,.0015f,800,.0001f,8500,425,8075));
    foc_current_feedforward(&s,2000,.0015f,.0015f,.00763f);
    for(unsigned k=0;k<1000;++k)
    {
        assert(foc_current_step(&s,0,0,1,0,1,0,12,0,.5f,out));
        assert(s.integral_d==0 && s.integral_q==0);
        assert(hypotf(s.command_d_v,s.command_q_v)<=12*.9f*.57735027f+.00001f);
        for(unsigned p=0;p<3;++p) { assert(out[p]>=425 && out[p]<=8075); }
    }
    assert(s.limited==1000U);
    foc_current_feedforward(&s,0,.0015f,.0015f,.00763f);
    assert(foc_current_step(&s,0,0,1,0,1,0,12,0,0,out));
    assert(out[0]==4250 && out[1]==4250 && out[2]==4250);
    foc_current_feedforward(&s,NAN,.0015f,.0015f,.00763f);
    assert(!foc_current_step(&s,0,0,1,0,1,0,12,0,0,out));
}

/** @brief 独立旋转RL/BEMF植物、真实CCR及两段延迟比较速度变化后的电流误差。
 * @param enabled 是否加入模型前馈。 @param model_scale 前馈参数比例。
 * @return 三次阶跃后的累计电流平方误差，不代表实板性能。 */
static float
feedforward_tracking(bool enabled,float model_scale)
{
    foc_current_t s;
    assert(foc_current_init(&s,10.1f,.0015f,800,.0001f,8500,425,8075));
    float alpha=0,beta=0,va=0,vb=0,theta=0,score=0,peak=0;
    for(unsigned k=0;k<2000;++k)
    {
        float omega=k<500?0:k<1500?400.0f:-400.0f;
        float reference=k<1000?.02f:.12f;
        if(enabled) { foc_current_feedforward(&s,omega,.0015f*model_scale,.0015f*model_scale,.00763f*model_scale); }
        uint16_t ccr[3];
        float future=theta+omega*.00010037647f;
        assert(foc_current_step(&s,alpha,beta,cosf(theta),sinf(theta),cosf(future),sinf(future),12,0,reference,ccr));
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        for(unsigned sub=0;sub<20;++sub)
        {
            float angle=theta+omega*((sub+.5f)*.000005f);
            float ea=-omega*.00763f*sinf(angle),eb=omega*.00763f*cosf(angle);
            /* 前50us仍为上一拍PWM，随后使用本拍请求，植物不读取内部PI电压。 */
            float applied_a=sub<10?va:next.alpha_v,applied_b=sub<10?vb:next.beta_v;
            alpha+=(applied_a-ea-10.1f*alpha)*(.000005f/.0015f);
            beta+=(applied_b-eb-10.1f*beta)*(.000005f/.0015f);
        }
        va=next.alpha_v;vb=next.beta_v;theta+=omega*.0001f;
        if(theta>6.2831853f) { theta-=6.2831853f; }
        if(theta<-6.2831853f) { theta+=6.2831853f; }
        float id=alpha*cosf(theta)+beta*sinf(theta),iq=-alpha*sinf(theta)+beta*cosf(theta);
        if(k>=500 && (k<550 || (k>=1000 && k<1050) || (k>=1500 && k<1550)))
        { score+=id*id+(iq-reference)*(iq-reference); }
        peak=fmaxf(peak,hypotf(id,iq));
        assert(isfinite(peak));
    }
    assert(peak<.6f);
    return score;
}

/** @brief 使用两段真实零阶保持RL植物验证控制延迟、参数误差及矢量限幅。
 * @param resistance 真实相电阻Ω。 @param inductance 真实相电感H。 */
static void
check_plant(float resistance,float inductance,float bandwidth)
{
    foc_current_t loop;
    assert(foc_current_init(&loop,.79f,.00024f,bandwidth,.0001f,8500,425,8075));
    float alpha=0,beta=0,old_alpha=0,old_beta=0;
    float a1=expf(-resistance*.00005037647f/inductance);
    float a2=expf(-resistance*.00004962353f/inductance);
    float peak=0;
    for (unsigned k=0;k<3000;++k)
    {
        uint16_t ccr[3];
        float d=k<1000?.2f:0, q=k>=1000&&k<2000?.2f:0;
        assert(foc_current_step(&loop,alpha,beta,.8660254f,.5f,.8660254f,.5f,12,d,q,ccr));
        for (unsigned p=0;p<3;++p) { assert(ccr[p]>=425&&ccr[p]<=8075); }
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        alpha=a1*alpha+(1-a1)*old_alpha/resistance;
        beta=a1*beta+(1-a1)*old_beta/resistance;
        alpha=a2*alpha+(1-a2)*next.alpha_v/resistance;
        beta=a2*beta+(1-a2)*next.beta_v/resistance;
        old_alpha=next.alpha_v;
        old_beta=next.beta_v;
        float magnitude=sqrtf(alpha*alpha+beta*beta);
        if(magnitude>peak) { peak=magnitude; }
        if(k==999||k==1999||k==2999)
        {
            assert(fabsf(loop.id_a-d)<.003f);
            assert(fabsf(loop.iq_a-q)<.003f);
        }
    }
    assert(peak<.28f);
}

/** @brief 旋转反电动势含5/7次谐波，检查50Hz强拖及未来作用角下的跟踪。
 * @param resistance 真实相电阻。 @param inductance 真实相电感。 */
static void
check_rotating(float resistance,float inductance)
{
    foc_current_t loop;
    assert(foc_current_init(&loop,.79f,.00024f,800,.0001f,8500,425,8075));
    float alpha=0,beta=0,old_alpha=0,old_beta=0,theta=0,omega=0,peak=0;
    for(unsigned k=0;k<45000;++k)
    {
        omega=fminf(314.159265f,omega+.0125663706f);
        theta+=omega*.0001f;
        if(theta>6.2831853f) { theta-=6.2831853f; }
        float future=theta+omega*.00010037647f;
        uint16_t ccr[3];
        assert(foc_current_step(&loop,alpha,beta,cosf(theta),sinf(theta),cosf(future),sinf(future),12,0,.3f,ccr));
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        for(unsigned half=0;half<2;++half)
        {
            float dt=half? .00004962353f:.00005037647f;
            float rotor=theta+omega*(half?.000075188235f:.000025188235f)-1.4f;
            float emf=omega*.008f;
            float ea=emf*(-sinf(rotor)+.10f*sinf(5*rotor)+.05f*sinf(7*rotor));
            float eb=emf*(cosf(rotor)+.10f*cosf(5*rotor)+.05f*cosf(7*rotor));
            float decay=expf(-resistance*dt/inductance);
            alpha=decay*alpha+(1-decay)*((half?next.alpha_v:old_alpha)-ea)/resistance;
            beta=decay*beta+(1-decay)*((half?next.beta_v:old_beta)-eb)/resistance;
        }
        old_alpha=next.alpha_v;
        old_beta=next.beta_v;
        float magnitude=sqrtf(alpha*alpha+beta*beta);
        if(magnitude>peak) { peak=magnitude; }
        assert(isfinite(magnitude));
    }
    fprintf(stderr,"rotating R=%.3f L=%.6f peak=%.4f\n",(double)resistance,(double)inductance,(double)peak);
    assert(peak<.48f);
    assert(loop.limited==0);
}

/** @brief 用独立含五次反电动势的延迟RL植物检验开关效果、增益偏差和角度纹波。
 * @param resistance 实际相电阻Ω。 @param inductance 实际相电感H。
 * @param frequency 电频率Hz。 @param angle_ripple 估计角六次纹波峰值rad。 */
static void
check_harmonic(float resistance,float inductance,float frequency,float angle_ripple)
{
    foc_current_t loop;
    assert(foc_current_init(&loop,.79f,.00024f,800,.0001f,8500,425,8075));
    float alpha=0,beta=0,old_alpha=0,old_beta=0,theta=0;
    float omega=6.28318530718f*frequency;
    double before_c=0,before_s=0,after_c=0,after_s=0;
    unsigned before_count=0,after_count=0;
    for (unsigned k=0;k<30000U;++k)
    {
        theta+=omega*.0001f;
        if (theta>6.28318530718f) { theta-=6.28318530718f; }
        float measured=theta+angle_ripple*sinf(6*theta);
        float future=measured+omega*.00010037647f;
        if (k==8000U) { foc_current_harmonic(&loop,true); }
        uint16_t ccr[3];
        assert(foc_current_step(&loop,alpha,beta,cosf(measured),sinf(measured),
            cosf(future),sinf(future),12,0,.124f,ccr));
        if (k>=2000U && k<8000U)
        {
            before_c+=alpha*cos(5*(double)theta);
            before_s+=alpha*sin(5*(double)theta);
            ++before_count;
        }
        if (k>=20000U)
        {
            after_c+=alpha*cos(5*(double)theta);
            after_s+=alpha*sin(5*(double)theta);
            ++after_count;
        }
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        for (unsigned half=0;half<2;++half)
        {
            float dt=half?.00004962353f:.00005037647f;
            float rotor=theta+omega*(half?.000075188235f:.000025188235f);
            float emf=2.545f*frequency/100.0f, harmonic=.0715f*frequency/100.0f;
            float ea=-emf*sinf(rotor)+harmonic*cosf(-5*rotor+.3f);
            float eb=emf*cosf(rotor)+harmonic*sinf(-5*rotor+.3f);
            float decay=expf(-resistance*dt/inductance);
            alpha=decay*alpha+(1-decay)*((half?next.alpha_v:old_alpha)-ea)/resistance;
            beta=decay*beta+(1-decay)*((half?next.beta_v:old_beta)-eb)/resistance;
        }
        old_alpha=next.alpha_v;
        old_beta=next.beta_v;
        assert(isfinite(alpha)&&isfinite(beta));
        if (k>2000U) { assert(hypotf(alpha,beta)<.3f); }
        assert(hypotf(loop.harmonic_x_v,loop.harmonic_y_v)<=.20001f);
    }
    double before=2*hypot(before_c,before_s)/before_count;
    double after=2*hypot(after_c,after_s)/after_count;
    fprintf(stderr,"harmonic R=%.3f L=%.6f Hz=%.0f angle=%.3f: %.3f -> %.3f mA\n",
        (double)resistance,(double)inductance,(double)frequency,(double)angle_ripple,before*1000,after*1000);
    assert(before>.01 && after<.003 && after<before*.15);
    assert(loop.limited==0 && loop.harmonic_limited==0);
    foc_current_harmonic(&loop,false);
    assert(loop.harmonic_x_v==0 && loop.harmonic_y_v==0);
    assert(loop.harmonic_d_v==0 && loop.harmonic_q_v==0);
}

/** @brief 独立物理植物同时含二/五/七次，保持五次ON比较二次OFF/ON/OFF。
 * @param resistance 实际相电阻Ω。 @param inductance 实际相电感H。
 * @param frequency 电频率Hz。 @param ripple 估计角三次纹波rad。 */
static void
check_even(float resistance,float inductance,float frequency,float ripple)
{
    foc_current_t loop;
    assert(foc_current_init(&loop,.79f,.00024f,800,.0001f,8500,425,8075));
    foc_current_harmonic(&loop,true);
    float alpha=0,beta=0,old_alpha=0,old_beta=0,theta=0;
    float omega=6.28318530718f*frequency;
    double sums[3][2]={{0}},fifth[3][2]={{0}};
    unsigned count[3]={0};
    for(unsigned k=0;k<40000U;++k)
    {
        theta+=omega*.0001f;
        if(theta>6.28318530718f) { theta-=6.28318530718f; }
        float measured=theta+ripple*sinf(3*theta),future=measured+omega*.00010037647f;
        if(k==10000U) { foc_current_harmonic2(&loop,true); }
        if(k==35000U) { foc_current_harmonic2(&loop,false); }
        uint16_t ccr[3];
        assert(foc_current_step(&loop,alpha,beta,cosf(measured),sinf(measured),
            cosf(future),sinf(future),12,0,.124f,ccr));
        int window=k>=4000U&&k<10000U?0:k>=25000U&&k<35000U?1:k>=36000U?2:-1;
        if(window>=0)
        {
            sums[window][0]+=alpha*cos(2*(double)theta);
            sums[window][1]+=alpha*sin(2*(double)theta);
            fifth[window][0]+=alpha*cos(5*(double)theta);
            fifth[window][1]+=alpha*sin(5*(double)theta);
            ++count[window];
        }
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        for(unsigned half=0;half<2;++half)
        {
            float dt=half?.00004962353f:.00005037647f;
            float rotor=theta+omega*(half?.000075188235f:.000025188235f);
            float scale=frequency/100.0f;
            float ea=scale*(-2.545f*sinf(rotor)+.0715f*cosf(-5*rotor+.3f)+
                           .0355f*cosf(-2*rotor+.7f)+.012f*cosf(7*rotor-.2f));
            float eb=scale*(2.545f*cosf(rotor)+.0715f*sinf(-5*rotor+.3f)+
                           .0355f*sinf(-2*rotor+.7f)+.012f*sinf(7*rotor-.2f));
            float decay=expf(-resistance*dt/inductance);
            alpha=decay*alpha+(1-decay)*((half?next.alpha_v:old_alpha)-ea)/resistance;
            beta=decay*beta+(1-decay)*((half?next.beta_v:old_beta)-eb)/resistance;
        }
        old_alpha=next.alpha_v;
        old_beta=next.beta_v;
        assert(isfinite(alpha)&&isfinite(beta));
        if(k>2000U) { assert(hypotf(alpha,beta)<.3f); }
        assert(hypotf(loop.harmonic2.x_v,loop.harmonic2.y_v)<=.10001f);
        for(unsigned p=0;p<3;++p) { assert(ccr[p]>=425&&ccr[p]<=8075); }
    }
    double amplitude[3];
    for(unsigned k=0;k<3;++k)
    {
        amplitude[k]=2*hypot(sums[k][0],sums[k][1])/count[k];
        assert(2*hypot(fifth[k][0],fifth[k][1])/count[k]<.003);
    }
    fprintf(stderr,"even R=%.3f L=%.6f Hz=%.0f angle=%.3f: %.3f -> %.3f -> %.3f mA\n",
        (double)resistance,(double)inductance,(double)frequency,(double)ripple,
        amplitude[0]*1000,amplitude[1]*1000,amplitude[2]*1000);
    assert(amplitude[0]>.002 && amplitude[1]<.003 && amplitude[1]<amplitude[0]*.25);
    assert(fabs(amplitude[2]-amplitude[0])<.002);
    assert(loop.limited==0 && loop.harmonic_limited==0 && loop.harmonic2.limited==0);
    assert(loop.harmonic2.x_v==0 && loop.harmonic2.y_v==0);
    assert(loop.harmonic2.d_v==0 && loop.harmonic2.q_v==0);
}

/** @brief 独立物理植物同时含二/五/七次，保持二/五次ON比较七次OFF/ON/OFF。
 * @param resistance 实际相电阻Ω。 @param inductance 实际相电感H。
 * @param frequency 电频率Hz。 @param ripple 估计角三次纹波rad。 */
static void
check_seventh(float resistance,float inductance,float frequency,float ripple)
{
    foc_current_t loop;
    assert(foc_current_init(&loop,.79f,.00024f,800,.0001f,8500,425,8075));
    foc_current_harmonic(&loop,true);
    foc_current_harmonic2(&loop,true);
    loop.harmonic7=(foc_harmonic2_t){0};
    float alpha=0,beta=0,old_alpha=0,old_beta=0,theta=0;
    float omega=6.28318530718f*frequency;
    double sums[3][2]={{0}},fifth[3][2]={{0}};
    unsigned count[3]={0};
    for(unsigned k=0;k<40000U;++k)
    {
        theta+=omega*.0001f;
        if(theta>6.28318530718f) { theta-=6.28318530718f; }
        float measured=theta+ripple*sinf(3*theta),future=measured+omega*.00010037647f;
        if(k==10000U) { loop.harmonic7=(foc_harmonic2_t){.enabled=true}; }
        if(k==35000U) { loop.harmonic7=(foc_harmonic2_t){0}; }
        uint16_t ccr[3];
        assert(foc_current_step(&loop,alpha,beta,cosf(measured),sinf(measured),
            cosf(future),sinf(future),12,0,.124f,ccr));
        int window=k>=4000U&&k<10000U?0:k>=25000U&&k<35000U?1:k>=36000U?2:-1;
        if(window>=0)
        {
            sums[window][0]+=alpha*cos(7*(double)theta);
            sums[window][1]+=alpha*sin(7*(double)theta);
            fifth[window][0]+=alpha*cos(5*(double)theta);
            fifth[window][1]+=alpha*sin(5*(double)theta);
            ++count[window];
        }
        foc_voltage_command_t next={.ccr={ccr[0],ccr[1],ccr[2]},.bus_v=12};
        assert(foc_voltage_prepare(&next,8500));
        for(unsigned half=0;half<2;++half)
        {
            float dt=half?.00004962353f:.00005037647f;
            float rotor=theta+omega*(half?.000075188235f:.000025188235f);
            float scale=frequency/100.0f;
            float ea=scale*(-2.545f*sinf(rotor)+.0715f*cosf(-5*rotor+.3f)+
                           .0355f*cosf(-2*rotor+.7f)+.012f*cosf(7*rotor-.2f));
            float eb=scale*(2.545f*cosf(rotor)+.0715f*sinf(-5*rotor+.3f)+
                           .0355f*sinf(-2*rotor+.7f)+.012f*sinf(7*rotor-.2f));
            float decay=expf(-resistance*dt/inductance);
            alpha=decay*alpha+(1-decay)*((half?next.alpha_v:old_alpha)-ea)/resistance;
            beta=decay*beta+(1-decay)*((half?next.beta_v:old_beta)-eb)/resistance;
        }
        old_alpha=next.alpha_v;
        old_beta=next.beta_v;
        assert(isfinite(alpha)&&isfinite(beta));
        if(k>2000U) { assert(hypotf(alpha,beta)<.3f); }
        assert(hypotf(loop.harmonic7.x_v,loop.harmonic7.y_v)<=.10001f);
        for(unsigned p=0;p<3;++p) { assert(ccr[p]>=425&&ccr[p]<=8075); }
    }
    double amplitude[3];
    for(unsigned k=0;k<3;++k)
    {
        amplitude[k]=2*hypot(sums[k][0],sums[k][1])/count[k];
        assert(2*hypot(fifth[k][0],fifth[k][1])/count[k]<.003);
    }
    fprintf(stderr,"seventh R=%.3f L=%.6f Hz=%.0f angle=%.3f: %.3f -> %.3f -> %.3f mA\n",
        (double)resistance,(double)inductance,(double)frequency,(double)ripple,
        amplitude[0]*1000,amplitude[1]*1000,amplitude[2]*1000);
    assert(amplitude[0]>.002 && amplitude[1]<.003 && amplitude[1]<amplitude[0]*.25);
    assert(fabs(amplitude[2]-amplitude[0])<.002);
    assert(loop.limited==0 && loop.harmonic_limited==0 && loop.harmonic2.limited==0 && loop.harmonic7.limited==0);
    assert(loop.harmonic7.x_v==0 && loop.harmonic7.y_v==0);
    assert(loop.harmonic7.d_v==0 && loop.harmonic7.q_v==0);
}

/** @brief 验证饱和解除后的恢复和接管积分矢量连续。 @return 成功0。 */
int
main(void)
{
    check_feedforward();
    float without=feedforward_tracking(false,1),with=feedforward_tracking(true,1);
    fprintf(stderr,"feedforward delayed plant: off=%.7f on=%.7f\n",(double)without,(double)with);
    assert(with<without*.6f);
    assert(feedforward_tracking(true,.8f)<without && feedforward_tracking(true,1.2f)<without);
    for(unsigned r=0;r<3;++r)
    {
        for(unsigned l=0;l<3;++l)
        {
            float resistance=.79f*(.8f+.2f*r),inductance=.00024f*(.8f+.2f*l);
            check_plant(resistance,inductance,200);
            check_plant(resistance,inductance,500);
            check_plant(resistance,inductance,800);
            check_rotating(resistance,inductance);
            check_harmonic(resistance,inductance,50,0);
            check_harmonic(resistance,inductance,100,0);
            check_harmonic(resistance,inductance,100,.015f);
            check_even(resistance,inductance,50,0);
            check_even(resistance,inductance,100,0);
            check_even(resistance,inductance,100,.01f);
            check_seventh(resistance,inductance,50,0);
            check_seventh(resistance,inductance,100,0);
            check_seventh(resistance,inductance,100,.01f);
        }
    }
    /* 梯形历史也受原饱和冻结约束：零指令不能重新灌入上一拍已拒绝的大误差。 */
    for(unsigned mode=0;mode<2;++mode)
    {
        foc_current_t guard;uint16_t bounded[3];
        assert(foc_current_init(&guard,.79f,.00024f,200,.0001f,8500,425,8075));
        foc_current_trapezoidal(&guard,mode!=0U);
        for(unsigned k=0;k<1000;++k)
        { assert(foc_current_step(&guard,0,0,1,0,1,0,12,100,100,bounded)); }
        assert(guard.limited>0 && guard.integral_d==0 && guard.integral_q==0);
        assert(guard.previous_error_d==0 && guard.previous_error_q==0);
        assert(foc_current_step(&guard,0,0,1,0,1,0,12,0,0,bounded));
        assert(bounded[0]==4250 && bounded[1]==4250 && bounded[2]==4250);
    }
    foc_current_t loop;
    assert(!foc_current_init(&loop,0,.00024f,200,.0001f,8500,425,8075));
    assert(foc_current_init(&loop,.79f,.00024f,200,.0001f,8500,425,8075));
    uint16_t ccr[3];
    for(unsigned k=0;k<1000;++k) { assert(foc_current_step(&loop,0,0,1,0,1,0,12,100,100,ccr)); }
    assert(loop.limited>0);
    assert(fabsf(loop.integral_d)<.001f&&fabsf(loop.integral_q)<.001f);
    assert(foc_current_step(&loop,0,0,1,0,1,0,12,0,0,ccr));
    assert(ccr[0]==4250&&ccr[1]==4250&&ccr[2]==4250);
    loop.integral_d=.2f;
    loop.integral_q=.1f;
    float d=loop.integral_d,q=loop.integral_q;
    foc_current_rotate(&loop,.8660254f,.5f);
    assert(fabsf((loop.integral_d*loop.integral_d+loop.integral_q*loop.integral_q)-(d*d+q*q))<.001f);
    assert(!foc_current_step(&loop,NAN,0,1,0,1,0,12,0,0,ccr));
    /* 总矢量饱和冻结谐波学习；谐波自身幅值限幅不能绕过原CCR窗口。 */
    assert(foc_current_init(&loop,.79f,.00024f,800,.0001f,8500,425,8075));
    foc_current_harmonic(&loop,true);
    foc_current_harmonic2(&loop,true);
    for (unsigned k=0;k<1000U;++k)
    {
        assert(foc_current_step(&loop,0,0,1,0,1,0,12,100,100,ccr));
        assert(loop.harmonic_x_v==0 && loop.harmonic_y_v==0);
        assert(loop.harmonic2.x_v==0 && loop.harmonic2.y_v==0);
        assert(loop.harmonic7.x_v==0 && loop.harmonic7.y_v==0);
        for (unsigned p=0;p<3;++p) { assert(ccr[p]>=425&&ccr[p]<=8075); }
    }
    puts("FOC current: delayed RL plants, parameter corners, dq steps, window and vector anti-windup PASS");
    return 0;
}
