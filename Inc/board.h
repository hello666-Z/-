/**
  ******************************************************************************
  * @file    board.h
  * @brief   Hardware pin map, wiring assumptions and every tuneable constant.
  *
  *  Board wiring recovered from the JLCEDA netlist
  *  (Netlist_Schematic1_2026-11-09.tel).  U1/U2 are the two 20-pin headers of
  *  the STM32F103C8T6 core board, therefore every net is named after the core
  *  board pin number it lands on:
  *
  *   net    header.pin  MCU pin  function here
  *   -----  ----------  -------  ------------------------------------------
  *   PWMA   U1.5        PA8      TIM1_CH1  motor A PWM  (H4.7)
  *   PWMB   U1.6        PA9      TIM1_CH2  motor B PWM  (H4.1)
  *   AIN1   U1.1        PB12     GPIO out  motor A direction (H4.5)
  *   AIN2   U1.2        PB13     GPIO out  motor A direction (H4.6)
  *   BIN1   U1.3        PB14     GPIO out  motor B direction (H4.3)
  *   BIN2   U1.4        PB15     GPIO out  motor B direction (H4.2)
  *   E1A    U2.10       PA6      TIM3_CH1  encoder 1 A
  *   E1B    U2.9        PA7      TIM3_CH2  encoder 1 B
  *   E2A    U1.14       PB6      TIM4_CH1  encoder 2 A
  *   E2B    U1.15       PB7      TIM4_CH2  encoder 2 B
  *   SCL    U2.6        PB10     I2C2_SCL  OLED (U5.3)
  *   SDA    U2.5        PB11     I2C2_SDA  OLED (U5.4)
  *   TXD    U2.14       PA2      USART2_TX bluetooth (U8.4)
  *   RXD    U2.13       PA3      USART2_RX bluetooth (U8.3)
  *   O1     U2.16       PA0      ADC1_IN0  amplifier output 1 (U4.6)
  *   O2     U2.15       PA1      ADC1_IN1  amplifier output 2 (U4.3)
  *   O3     U2.12       PA4      ADC1_IN4  amplifier output 3 (U4.2)
  *   O4     U2.11       PA5      ADC1_IN5  amplifier output 4 (U4.1)
  *   A      U1.12       PB4      EXTI4     rotary encoder A (U10.1)
  *   B      U1.13       PB5      EXTI5     rotary encoder B (U10.3)
  *
  *  H4 = motor driver module input, H5 = motor driver module output
  *  (E1A/E1B/E2A/E2B encoder feedback), U3/U11 = induction coil + amplifier,
  *  U4 = amplifier outputs to the MCU, U5 = OLED, U8 = bluetooth module,
  *  U10 (A/B) + U6 (3V3/GND) = rotary encoder.
  ******************************************************************************
  */
#ifndef __BOARD_H
#define __BOARD_H

#include "main.h"

/* ==========================================================================
 * PWM : TIM1 CH1/CH2, 72 MHz / 4200 counts = 17.14 kHz
 * Duty is limited to 7.4 V / 12.6 V = 58.7 % -> CCR 2400 of 4200 counts.
 * ========================================================================== */
#define PWM_PERIOD_COUNTS   4200U       /* ARR + 1                     */
#define PWM_ARR             4199U       /* ARR                          */
#define PWM_FREQ_HZ         17143U      /* 72 MHz / 4200                */
#define PWM_FULL_SCALE      4200.0f     /* counts for 100 % duty        */
#define PWM_MAX_CCR         2400U       /* ~58 % duty, never exceeded   */
#define PWM_MAX_DUTY_PCT    57.1f       /* PWM_MAX_CCR / PWM_PERIOD     */
#define PWM_MIN_CCR         150U        /* dead zone compensation       */

/* ==========================================================================
 * Motor driver (TB6612 style: IN1/IN2 direction + PWM on xPWMA)
 * ========================================================================== */
#define MOTOR_A_IS_LEFT     1U          /* H4 channel A drives the left wheel */
#define MOTOR_A_INVERT      0U          /* flip if the wheel spins backwards  */
#define MOTOR_B_INVERT      0U

#define AIN1_GPIO_Port  GPIOB
#define AIN1_Pin        GPIO_PIN_12
#define AIN2_GPIO_Port  GPIOB
#define AIN2_Pin        GPIO_PIN_13
#define BIN1_GPIO_Port  GPIOB
#define BIN1_Pin        GPIO_PIN_14
#define BIN2_GPIO_Port  GPIOB
#define BIN2_Pin        GPIO_PIN_15

/* small feed forward so the speed loop starts close to the right duty.
   Output unit: PWM counts per (mm/s).  0 disables it.                    */
#define MOTOR_FF_CCR_PER_MMS  1.0f

/* ==========================================================================
 * Encoders : TIM3 (E1) and TIM4 (E2), hardware quadrature, 4 counts/line
 * counts per wheel revolution = lines * 4 * gearbox ratio
 * ========================================================================== */
#define ENC_TIM_LEFT        htim3       /* E1A/E1B -> PA6/PA7          */
#define ENC_TIM_RIGHT       htim4       /* E2A/E2B -> PB6/PB7          */
#define ENC_COUNTS_PER_REV  1560.0f     /* 13 PPR * 4 * 30:1 gearbox   */
#define WHEEL_DIAM_MM       65.0f
#define WHEEL_BASE_MM       150.0f      /* distance between the wheels */
#define ENC_LEFT_SIGN       (+1.0f)     /* set -1.0f if a wheel counts backwards */
#define ENC_RIGHT_SIGN      (+1.0f)

/* ==========================================================================
 * Control loop
 * ========================================================================== */
#define CONTROL_PERIOD_MS   10U         /* 100 Hz speed/direction loop  */
#define CONTROL_DT_S        0.010f
#define TELEMETRY_PERIOD_MS 100U
#define OLED_PERIOD_MS      50U         /* 20 Hz display refresh        */

/* ---- automatic (induction line following) ---- */
#define AUTO_SPEED_MMS      450.0f      /* base speed of both wheels    */
#define AUTO_SPEED_STEP     50.0f       /* +/- adjustment per command   */
#define AUTO_SPEED_MIN      120.0f
#define AUTO_SPEED_MAX      900.0f
#define AUTO_START_DELAY_MS 1000U       /* arm the car this long after 'a' */

/* ---- direction loop : induction error (-1..+1) -> speed difference ---- */
#define DIR_KP              300.0f
#define DIR_KI              80.0f
#define DIR_KD              0.0f
#define DIR_OUT_MAX_MMS     400.0f      /* max differential speed       */
#define DIR_I_MAX_MMS       300.0f

/* ---- speed loop : wheel speed error (mm/s) -> PWM counts ---- */
#define SPEED_KP            4.0f
#define SPEED_KI            12.0f
#define SPEED_KD            0.0f
#define SPEED_I_MAX_CCR     1800.0f
#define SPEED_LPF           0.35f       /* encoder speed low pass, 0..1 */

/* ---- manual mode ---- */
#define MAN_SPEED_MAX_MMS   900.0f      /* 100 % manual throttle        */
#define MAN_DEADBAND_PCT    3

/* ==========================================================================
 * Induction sensor  (ADC1 scan + DMA, 4 channels)
 * ========================================================================== */
#define EC_CHANNELS         4U
#define EC_LPF              0.25f       /* per-sample low pass on ADC values */
#define EC_SIGNAL_MIN       120U        /* raw counts: below = no track */
#define EC_ERR_GAIN         1.0f        /* error = gain * (L-R)/(L+R)   */

/*  Weight of each ADC channel in the difference-over-sum error.
 *  O1 (PA0) and O2 (PA1) are the horizontal pair by default; O3/O4 may be
 *  wired as a second horizontal pair or as the vertical pair, so they are
 *  disabled (0.0f) until the raw values on the OLED prove otherwise.
 *  Left channels add, right channels subtract.                            */
#define EC_W_L1             1.0f        /* O1 PA0 */
#define EC_W_R1             1.0f        /* O2 PA1 */
#define EC_W_L2             0.0f        /* O3 PA4 */
#define EC_W_R2             0.0f        /* O4 PA5 */

#define EC_BASELINE_SAMPLES 64U         /* boot calibration length      */
#define EC_TRACK_LOST_MS    500U        /* no signal this long -> stop  */
#define EC_LOST_ACTION_STOP 1U          /* 1 = brake the car            */

/* ==========================================================================
 * Lap counting : integrate the heading rate of the differential drive
 * ========================================================================== */
#define LAP_ENABLE          1U
#define LAP_STOP_AFTER      0U          /* 1 = brake after LAP_TARGET_LAPS */
#define LAP_TARGET_LAPS     1
#define LAP_ANGLE_DEG       360.0f      /* one full circle              */

/* ==========================================================================
 * Rotary encoder knob : PB4/PB5, quadrature through EXTI
 * ========================================================================== */
#define KNOB_STEP_PCT       5           /* throttle change per detent   */
#define KNOB_MAX_PCT        100
#define KNOB_IDLE_MS        5000U       /* manual takeover timeout note */

#define KNOB_A_GPIO_Port    GPIOB       /* U10.1, 4.7k pull-up on board */
#define KNOB_A_Pin          GPIO_PIN_4
#define KNOB_B_GPIO_Port    GPIOB       /* U10.3                        */
#define KNOB_B_Pin          GPIO_PIN_5

/* ==========================================================================
 * Bluetooth : USART2, line oriented ASCII protocol (see bt.h)
 * ========================================================================== */
#define BT_BAUDRATE         9600U       /* HC-05/HC-06/AT-09 default    */
#define BT_RING_SIZE        128U

/* ==========================================================================
 * OLED : SSD1306 128x64 on I2C2
 * ========================================================================== */
#define OLED_I2C_ADDR       (0x3C << 1) /* 0x78: SA0 low (0x7A if SA0 high) */
#define OLED_WIDTH          128U
#define OLED_HEIGHT         64U
#define OLED_PAGES          (OLED_HEIGHT / 8U)

#endif /* __BOARD_H */
