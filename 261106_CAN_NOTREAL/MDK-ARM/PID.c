#include "pid.h"
void PID_Init(PID_t *p, float kp, float ki, float kd,
              float out_max, float out_min, float i_max)
{
    p->kp = kp;  p->ki = ki;  p->kd = kd;
    p->out_max = out_max;
    p->out_min = out_min;
    p->i_max   = i_max;

    p->err = 0;
    p->err_last = 0;
    p->integral = 0;
    p->output = 0;
}
void PID_Calc(PID_t *p, float target, float feedback, float dt){
	p -> err = target - feedback;
	p ->integral = p -> integral + p->err * dt;
	if( p->integral > p->i_max ){p->integral = p->i_max;}
	if( p -> integral < -p -> i_max){p -> integral = -p->i_max;}
	p->output = p->kp * p->err + p->ki * p->integral + p->kd * (p->err - p-> err_last) / dt;
	if( p->output > p->out_max){p->output = p->out_max;}
	if( p->output < p->out_min){p->output = p->out_min;}
	p->err_last = p->err;
}