/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Wheel speed from the two hardware quadrature timers (TIM3/TIM4).
  *
  * Both timers run in encoder mode TI1+TI2 (4 counts per encoder line), 16 bit
  * free running.  The counter difference between two control periods is taken
  * as int16_t so a wraparound is handled automatically.
  ******************************************************************************
  */
#include "encoder.h"
#include "board.h"
#include "tim.h"

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

static uint16_t s_prev_l;
static uint16_t s_prev_r;
static int32_t  s_delta_l;
static int32_t  s_delta_r;
static float    s_vl;
static float    s_vr;
static float    s_distance_mm;
static float    s_mm_per_count;

void Encoder_Init(void)
{
  (void)HAL_TIM_Encoder_Start(&ENC_TIM_LEFT, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&ENC_TIM_RIGHT, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&ENC_TIM_LEFT, 0);
  __HAL_TIM_SET_COUNTER(&ENC_TIM_RIGHT, 0);

  s_prev_l = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM_LEFT);
  s_prev_r = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM_RIGHT);
  s_delta_l = 0;
  s_delta_r = 0;
  s_vl = 0.0f;
  s_vr = 0.0f;
  s_distance_mm = 0.0f;

  s_mm_per_count = (M_PI * WHEEL_DIAM_MM) / ENC_COUNTS_PER_REV;
}

void Encoder_Update(void)
{
  uint16_t now_l = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM_LEFT);
  uint16_t now_r = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM_RIGHT);
  float raw_l;
  float raw_r;

  s_delta_l = (int32_t)(int16_t)(uint16_t)(now_l - s_prev_l);
  s_delta_r = (int32_t)(int16_t)(uint16_t)(now_r - s_prev_r);
  s_prev_l = now_l;
  s_prev_r = now_r;

  raw_l = ((float)s_delta_l * s_mm_per_count) / CONTROL_DT_S;
  raw_r = ((float)s_delta_r * s_mm_per_count) / CONTROL_DT_S;

  s_vl += (raw_l - s_vl) * SPEED_LPF;
  s_vr += (raw_r - s_vr) * SPEED_LPF;

  s_distance_mm += 0.5f * (s_vl + s_vr) * CONTROL_DT_S;
}

float Encoder_LeftMms(void)
{
  return s_vl * ENC_LEFT_SIGN;
}

float Encoder_RightMms(void)
{
  return s_vr * ENC_RIGHT_SIGN;
}

float Encoder_AverageMms(void)
{
  return 0.5f * (Encoder_LeftMms() + Encoder_RightMms());
}

int32_t Encoder_LeftDelta(void)
{
  return s_delta_l;
}

int32_t Encoder_RightDelta(void)
{
  return s_delta_r;
}

float Encoder_DistanceMm(void)
{
  return s_distance_mm;
}
