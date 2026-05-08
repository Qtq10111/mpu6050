//
// Created by 屈天齐 on 2026/5/3.
//

#include "ENCOUNT.h"
#include "tim.h"
//#include "PID.h"
#include "stm32f4xx.h"



void ENCOUNT_INIT(encoder *CODER1,encoder *CODER2) {
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim2);
    __HAL_TIM_SetCounter(&htim3,10000);
    __HAL_TIM_SetCounter(&htim4,10000);
    CODER1->lastcount = 10000;
    CODER2->lastcount = 10000;
    CODER1->totalcount = 0;
    CODER2->totalcount = 0;
    CODER1->overflowNUM = 0;
    CODER2->overflowNUM = 0;
}

float Speed_calculate(float t, float multi,int PPR, int GR ,float wheel,encoder *coder,int count) {
    coder->totalcount = count + coder->overflowNUM * 20000;
    float Pulse = PPR * multi * GR;
    float arti_speed = ((coder->totalcount-coder->lastcount) * wheel)/(t * Pulse);
    coder->lastcount = coder->totalcount;
    return arti_speed;
}