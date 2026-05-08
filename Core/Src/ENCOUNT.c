//
// Created by 屈天齐 on 2026/5/3.
//

#include "ENCOUNT.h"
#include "tim.h"
//#include "PID.h"
#include "stm32f4xx.h"
#include "tim.h"

#define  tim_arr_value 20000
#define  tim_half_arr (tim_arr_value / 2)

void ENCOUNT_INIT(encoder *CODER1,encoder *CODER2) {
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim2);
    __HAL_TIM_SetCounter(&htim3,tim_half_arr);
    __HAL_TIM_SetCounter(&htim4,tim_half_arr);
    CODER1->lastcount = tim_half_arr;
    CODER2->lastcount = tim_half_arr;
    CODER1->speed = 0.0f;
    CODER2->speed = 0.0f;
    // CODER1->totalcount = 0;
    // CODER2->totalcount = 0;
    // CODER1->overflowNUM = 0;
    // CODER2->overflowNUM = 0;
}

float Speed_calculate(float t, float multi,int PPR, int GR ,float wheel,encoder *coder,TIM_HandleTypeDef *htim) {
    uint16_t cnt_now = __HAL_TIM_GetCounter(htim);
    int32_t delta = cnt_now - coder->lastcount;
    if (delta > tim_half_arr) {
        delta -= tim_arr_value;
    } else if (delta < -tim_half_arr) {
        delta += tim_arr_value;
    }
    coder->lastcount = cnt_now;
    float pulse_per_rev = (float)PPR * multi * (float)GR;
    float speed_rps = (float)delta / pulse_per_rev / t;
    float speed_mps = speed_rps * wheel;

    const float alpha = 0.3f;
    coder->speed = alpha * speed_mps + (1.0f - alpha) * coder->speed;

    return coder->speed;
    // coder->totalcount = count + coder->overflowNUM * 20000;
    // float Pulse = PPR * multi * GR;
    // float arti_speed = ((coder->totalcount-coder->lastcount) * wheel)/(t * Pulse);
    // coder->lastcount = coder->totalcount;
    // return arti_speed;
}