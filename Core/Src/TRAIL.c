//
// Created by 屈天齐 on 2026/5/7.
//

#include "TRAIL.h"
#include "stm32f4xx.h"
#include "tim.h"
void trial_config(Tracking_PID_t *trial,float kp,float kd,float basespeed) {
    trial->Kp = kp;
    trial->Kd = kd;
    trial->BaseSpeed = basespeed;
    trial->last_error = 0;
}

void Motor_config(Motor *motor,uint16_t motor_pwmr,uint16_t motor_pwml,enum DIR direction) {
    motor->motor_pwmr = motor_pwmr;
    motor->motor_pwml = motor_pwml;
    motor->dir = direction;
}

void trial_main(Motor *motor,Tracking_PID_t *trial,uint8_t val){
    float error = 0;
    int active_count = 0;
    float weights[8] = {-13.0f, -6.0f, -3.0f, -1.0f, 1.0f, 3.0f, 6.0f, 13.0f};

    if (val == 0) {//未检测到黑线则停止
        motor->dir = STOP;
        MOTOR_RUN(motor);
        return;
    }

    for (int i = 0; i < 8; i++) {//计算权重决定转向的速度
        // 判断当前位置(左)/位置(右)的传感器状态
        if ((val >> (7 - i)) & 0x01) {
            error += weights[i];
            active_count ++;
        }
    }

    if (active_count > 0) {
        error /= active_count;
    }

    float output = trial->Kp * error + trial->Kd * (error - trial->last_error);//pd控制
    trial->last_error = error;
   //计算速度值
    int16_t leftspeed = trial->BaseSpeed + (int16_t)output;
    int16_t rightspeed = trial->BaseSpeed - (int16_t)output;
    //限幅
    if (leftspeed > 999) leftspeed = 999;
    if (leftspeed < -999) leftspeed = -999;
    if (rightspeed > 999) rightspeed = 999;
    if (rightspeed < -999) rightspeed = -999;

    if (leftspeed != 0 && rightspeed != 0) {
        if (leftspeed == rightspeed) {
            Motor_config(motor,rightspeed,leftspeed,STRIGHT);
        }else if (leftspeed > rightspeed) {
            Motor_config(motor,leftspeed,rightspeed,TURN_RIGHT);
        }else if (leftspeed < rightspeed) {
            Motor_config(motor,leftspeed,rightspeed,TURN_LEFT);
        }
    }
    else {
        Motor_config(motor,rightspeed,leftspeed,STOP);
    }
    MOTOR_RUN(motor);

}

void MOTOR_RUN(Motor *motor) {
    if (motor->dir == STOP) {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
    }
    else if (motor->dir == TURN_RIGHT) {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->motor_pwml);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->motor_pwmr);
    }
    else if (motor->dir == TURN_LEFT) {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->motor_pwml);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->motor_pwmr);
    }else if (motor->dir == STRIGHT) {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor->motor_pwml);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor->motor_pwmr);
    }



}

uint8_t trial_read(void) {
    uint8_t trial[8];
    uint8_t result = 0;
    trial[0] =  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0);
    trial[1] =  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1);
    trial[2] =  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2);
    trial[3] =  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2);
    trial[4] =  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3);
    trial[5] =  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4);
    trial[6] =  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5);
    trial[7] =  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6);

    result |= (trial[0] ? 0x80 : 0x00);  // IR1 -> bit7
    result |= (trial[1] ? 0x40 : 0x00);  // IR2 -> bit6
    result |= (trial[2] ? 0x20 : 0x00);  // IR3 -> bit5
    result |= (trial[3] ? 0x10 : 0x00);  // IR4 -> bit4
    result |= (trial[4] ? 0x08 : 0x00);  // IR5 -> bit3
    result |= (trial[5] ? 0x04 : 0x00);  // IR6 -> bit2
    result |= (trial[6] ? 0x02 : 0x00);  // IR7 -> bit1
    result |= (trial[7] ? 0x01 : 0x00);  // IR8 -> bit0

    return  result;
}