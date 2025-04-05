#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f10x.h"

#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);
void Encoder_Init_TIM5(void);
int Read_Encoder(u8 TIMX);

int16_t Encoder_Get1(void);
int16_t Encoder_Get2(void);
int16_t Encoder_Get3(void);
int16_t Encoder_Get4(void);


//void TIM2_IRQHandler(void);
//void TIM3_IRQHandler(void);
//void TIM4_IRQHandler(void);
//void TIM5_IRQHandler(void);
#endif
