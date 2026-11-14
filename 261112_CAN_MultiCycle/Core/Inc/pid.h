#ifndef __PID_H
#define __PID_H
typedef struct {
	float kp, ki, kd;
	float out_max, out_min;
	float i_max;
	float err;
	float err_last;
	float integral;/* 误差累积和 */ 
	float output;
}PID_t;
void PID_Init(PID_t* p, float kp, float ki, float kd, float out_max, float out_min, float i_max);
void PID_Calc(PID_t* p, float target, float feedback, float dt);

#endif

