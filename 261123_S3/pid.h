#ifndef _PID_H
#define _PID_H
#include <stdint.h>
typedef struct {
	float kp, ki, kd;
	float out_max, out_min, i_max;
	float err, err_last, integral;
	float output;
}PID_t;
void PID_Init(PID_t* p, float kp, float ki, float kd, float out_max, float out_min, float i_max);
void PID_Calc(PID_t* p, float target, float feedback, float dt);
#endif