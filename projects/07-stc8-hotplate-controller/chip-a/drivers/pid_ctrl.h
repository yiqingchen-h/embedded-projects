#ifndef PID_CTRL_H
#define PID_CTRL_H

typedef struct
{
    signed int kp_x100;
    signed int ki_x100;
    signed int kd_x100;
    signed long integral_accum;
    signed int prev_error_c10;
    unsigned char initialized;
} pid_ctrl_t;

void PIDCTRL_Init(pid_ctrl_t *pid, signed int kp_x100, signed int ki_x100, signed int kd_x100);
void PIDCTRL_Reset(pid_ctrl_t *pid);
unsigned char PIDCTRL_Compute(pid_ctrl_t *pid, signed int set_temp_c10, signed int cur_temp_c10);

#endif /* PID_CTRL_H */
