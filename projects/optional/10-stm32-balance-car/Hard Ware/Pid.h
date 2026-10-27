
#ifndef __PID_H
#define __PID_H

int vertical_PID_value(float measure,float calcu); //直立环
int velocity_PID_value(int velocity);              //速度环
int ZhuanXiang_Pid_Value(short gyro ,float MuBiao_Jiao);			//转向换
int velocity_PID_value(int velocity);                         //pwm限幅

#endif
