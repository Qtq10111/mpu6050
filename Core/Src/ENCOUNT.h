//
// Created by 屈天齐 on 2026/5/3.
//

#ifndef MPU6050_ENCOUNT_H
#define MPU6050_ENCOUNT_H
#include  "stdint.h"
#include "stm32f4xx.h"


typedef struct {//需要在主函数里初始化两个编码器实例
    int lastcount;
    int totalcount;
    int overflowNUM;
}encoder;


void ENCOUNT_INIT(encoder *CODER1,encoder *CODER2);//编码器和pwm初始化
float Speed_calculate(float t, float multi,int PPR, int GR ,float wheel,encoder *coder,int count);
#endif //MPU6050_ENCOUNT_H