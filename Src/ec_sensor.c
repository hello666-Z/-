/**
  ******************************************************************************
  * @file    ec_sensor.c
  * @brief   Induction line sensor front end.
  ******************************************************************************
  */
#include "ec_sensor.h"
#include "board.h"
#include "adc.h"

static uint16_t s_dma[EC_CHANNELS];
static float    s_filt[EC_CHANNELS];
static float    s_base[EC_CHANNELS];
static float    s_cal_sum[EC_CHANNELS];
static uint32_t s_cal_n;
static uint8_t  s_cal_busy;
static uint8_t  s_cal_done;

static float    s_err;
static float    s_strength;
static uint8_t  s_on_track;

static float rel(uint8_t ch)
{
  float v = s_filt[ch] - s_base[ch];
  return (v > 0.0f) ? v : 0.0f;
}

void EC_Init(void)
{
  uint8_t i;

  for (i = 0U; i < EC_CHANNELS; i++)
  {
    s_dma[i] = 0U;
    s_filt[i] = 0.0f;
    s_base[i] = 0.0f;
    s_cal_sum[i] = 0.0f;
  }
  s_cal_n = 0U;
  s_cal_busy = 0U;
  s_cal_done = 0U;
  s_err = 0.0f;
  s_strength = 0.0f;
  s_on_track = 0U;

  (void)HAL_ADCEx_Calibration_Start(&hadc1);
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma, EC_CHANNELS);
}

void EC_Calibrate(void)
{
  uint8_t i;

  for (i = 0U; i < EC_CHANNELS; i++)
  {
    s_cal_sum[i] = 0.0f;
  }
  s_cal_n = 0U;
  s_cal_busy = 1U;
}

uint8_t EC_Calibrated(void)
{
  return s_cal_done;
}

void EC_Update(void)
{
  uint8_t i;
  float left = 0.0f;
  float right = 0.0f;
  float sum;
  float best = 0.0f;

  /* low pass every raw channel to kill the 17 kHz motor PWM pick up */
  for (i = 0U; i < EC_CHANNELS; i++)
  {
    s_filt[i] += ((float)s_dma[i] - s_filt[i]) * EC_LPF;
  }

  if (s_cal_busy != 0U)
  {
    for (i = 0U; i < EC_CHANNELS; i++)
    {
      s_cal_sum[i] += s_filt[i];
    }
    s_cal_n++;
    if (s_cal_n >= EC_BASELINE_SAMPLES)
    {
      for (i = 0U; i < EC_CHANNELS; i++)
      {
        s_base[i] = s_cal_sum[i] / (float)s_cal_n;
      }
      s_cal_busy = 0U;
      s_cal_done = 1U;
    }
    s_err = 0.0f;
    return;
  }

  left  = (EC_W_L1 * rel(0U)) + (EC_W_L2 * rel(2U));
  right = (EC_W_R1 * rel(1U)) + (EC_W_R2 * rel(3U));
  sum = left + right;

  if (sum > 1.0f)
  {
    s_err = EC_ERR_GAIN * (left - right) / sum;
  }
  else
  {
    s_err = 0.0f;
  }

  for (i = 0U; i < EC_CHANNELS; i++)
  {
    float v = rel(i);
    if (v > best)
    {
      best = v;
    }
  }
  s_strength = best;
  s_on_track = (best > (float)EC_SIGNAL_MIN) ? 1U : 0U;
}

float EC_Error(void)
{
  return s_err;
}

float EC_Strength(void)
{
  return s_strength;
}

uint8_t EC_OnTrack(void)
{
  return s_on_track;
}

uint16_t EC_Raw(uint8_t ch)
{
  return (ch < EC_CHANNELS) ? s_dma[ch] : 0U;
}

uint16_t EC_Relative(uint8_t ch)
{
  float v;

  if (ch >= EC_CHANNELS)
  {
    return 0U;
  }
  v = rel(ch);
  return (v > 9999.0f) ? 9999U : (uint16_t)v;
}

uint16_t EC_Baseline(uint8_t ch)
{
  return (ch < EC_CHANNELS) ? (uint16_t)s_base[ch] : 0U;
}
