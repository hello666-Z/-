/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Wheel speed from the two hardware quadrature timers (TIM3/TIM4).
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void  Encoder_Init(void);
void  Encoder_Update(void);          /* call every CONTROL_PERIOD_MS */

float Encoder_LeftMms(void);         /* signed, + = forward */
float Encoder_RightMms(void);
float Encoder_AverageMms(void);
int32_t Encoder_LeftDelta(void);     /* counts during the last period */
int32_t Encoder_RightDelta(void);
float Encoder_DistanceMm(void);      /* integral of the average speed */

#endif /* __ENCODER_H */
