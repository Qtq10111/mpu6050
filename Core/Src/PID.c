//
// Created by 屈天齐 on 2026/5/3.
//

#include "PID.h"
void PID_init(PID *pid,float p,float i,float d,float maxI,float maxO){
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxoutput = maxO;
    pid->maxintegral = maxI;
    pid->error = 0.0f;
    pid->lasterror = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void PID_CALC(PID *pid,float reference,float feedback){
    pid->lasterror = pid->error;
    pid->error = reference - feedback;
    pid->output = (pid->error - pid->lasterror) * pid->kd;
    pid->output += pid->error * pid->kp;
    pid->integral += pid->error * pid->ki;
    if(pid->integral >= pid->maxintegral){
        pid->integral = pid->maxintegral;
    }else if(pid->integral <= -pid->maxintegral){
        pid->integral = -pid->maxintegral;
    }
    pid->output += pid->integral;
    if(pid->output >= pid->maxoutput){
        pid->output = pid->maxoutput;
    }else if(pid->output <= -pid->maxoutput){
        pid->output = -pid->maxoutput;
    }
}