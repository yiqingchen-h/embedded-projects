#ifndef __PID_H
#define __PID_H

extern float Speed_Ki, Speed_Kp, Speed_Kd;
extern float Places_Ki, Places_Kp, Places_Kd;
short Pid_Speed(short Speed, short Target);
short Pid_places(short Places, short Target);
short Pid_Speed_places(short Speed, short Places, short Target_Places);

// 倒立摆位置环
short PID_Place(short Place, short Target);

#endif
