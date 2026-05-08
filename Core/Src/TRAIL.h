//
// Created by 屈天齐 on 2026/5/7.
//

#ifndef MPU6050_TRAIL_H
#define MPU6050_TRAIL_H

#include "stdint.h"


enum  DIR {//电机的三种运动状态，后续可以放在灰度里
    TURN_LEFT,
    TURN_RIGHT,
    STRIGHT,
    STOP
};

typedef struct {//需要在主函数里初始化一个循迹实例
    float Kp;
    float Kd;
    int16_t BaseSpeed;
    float last_error;
} Tracking_PID_t;

typedef struct {
    uint16_t motor_pwmr,motor_pwml;
    enum DIR dir;
}Motor;//需要在主函数里初始化一个马达实例
void trial_config(Tracking_PID_t *trial,float kp,float kd,float basespeed);//循迹的参数设置
void Motor_config(Motor *motor,uint16_t motor_pwmr,uint16_t motor_pwml,enum DIR direction);//配置校车运动状态
void MOTOR_RUN(Motor *motor);
uint8_t trial_read(void);//读取灰度传感器数值，仅在库内部调用
void trial_main(Motor *motor,Tracking_PID_t *trial,uint8_t val);
#endif //MPU6050_TRAIL_H