#ifndef FOC_CONTROL_FOC_MATH_H
#define FOC_CONTROL_FOC_MATH_H

/** @brief 在已归一化电角度上同时求正余弦；输入范围[-2pi,2pi]。
 * @param angle 电角度rad。 @param cosine 余弦输出。 @param sine 正弦输出。
 * @note 先折叠到[-pi/2,pi/2]再13/12阶展开，完整角度扫频独立核对误差。
 * 只使用单精度乘加，不调用数百周期的重复角度归约库函数。 */
static inline void
foc_sincos(float angle,float *cosine,float *sine)
{
    if(angle>3.14159265359f) { angle-=6.28318530718f; }
    if(angle<-3.14159265359f) { angle+=6.28318530718f; }
    float sign=1;
    if(angle>1.57079632679f) { angle=3.14159265359f-angle; sign=-1; }
    if(angle<-1.57079632679f) { angle=-3.14159265359f-angle; sign=-1; }
    float x2=angle*angle;
    *sine=angle*(1+x2*(-.166666666667f+x2*(.00833333333333f+x2*(-.000198412698413f+
        x2*(.00000275573192240f+x2*(-.0000000250521083854f+x2*.000000000160590438368f))))));
    *cosine=sign*(1+x2*(-.5f+x2*(.0416666666667f+x2*(-.00138888888889f+
        x2*(.0000248015873016f+x2*(-.00000027557319224f+x2*.00000000208767569879f))))));
}
#endif
