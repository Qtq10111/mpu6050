//
// Created by 屈天齐 on 2026/5/3.
//

#ifndef MPU6050_PID_H
#define MPU6050_PID_H
typedef struct {
    float kp,ki,kd;
    float error,lasterror;
    float integral,maxintegral;
    float output,maxoutput;
}PID;


void PID_init(PID *pid,float p,float i,float d,float maxI,float maxO);

void PID_CALC(PID *pid,float reference,float feedback);
#endif //MPU6050_PID_H