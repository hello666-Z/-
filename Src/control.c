/**
  ******************************************************************************
  * @file    control.c
  * @brief   Cascaded control: induction direction loop -> wheel speed loop.
  ******************************************************************************
  */
#include "control.h"
#include "board.h"
#include "motor.h"
#include "encoder.h"
#include "ec_sensor.h"
#include "knob.h"
#include "bt.h"
#include "oled.h"
#include "pid.h"
#include <stdio.h>

#define RAD_TO_DEG 57.2957795f

static Pid_t   s_pid_spd_l;
static Pid_t   s_pid_spd_r;
static Pid_t   s_pid_dir;

static CtrlMode_t s_mode;
static float   s_auto_speed;
static float   s_steer;
static float   s_tgt_l;
static float   s_tgt_r;
static int32_t s_u_l;
static int32_t s_u_r;
static int32_t s_lap;
static float   s_heading_deg;
static uint32_t s_lost_ms;
static uint32_t s_arm_at;
static uint32_t s_telem_at;
static uint8_t s_armed;
static uint8_t s_lap_done;
static uint8_t s_hold;
static uint8_t s_track_lost;
static uint8_t s_telem_on;
static uint8_t s_autocal_done;

/* ------------------------------------------------------------------------- */
static void reset_loops(void)
{
  Pid_Reset(&s_pid_spd_l);
  Pid_Reset(&s_pid_spd_r);
  Pid_Reset(&s_pid_dir);
  s_steer = 0.0f;
  s_tgt_l = 0.0f;
  s_tgt_r = 0.0f;
  s_u_l = 0;
  s_u_r = 0;
}

static void enter_auto(void)
{
  s_mode = CTRL_MODE_AUTO;
  reset_loops();
  s_auto_speed = AUTO_SPEED_MMS;
  s_heading_deg = 0.0f;
  s_lap = 0;
  s_lap_done = 0U;
  s_lost_ms = 0U;
  s_track_lost = 0U;
  s_armed = 0U;
  s_arm_at = HAL_GetTick() + AUTO_START_DELAY_MS;
}

static void enter_manual(CtrlMode_t src)
{
  if (s_mode != src)
  {
    reset_loops();
  }
  s_mode = src;
  s_lap_done = 0U;
  s_hold = 0U;
  s_armed = 1U;
}

void Control_Init(void)
{
  Pid_Init(&s_pid_spd_l, SPEED_KP, SPEED_KI, SPEED_KD,
           -(float)PWM_MAX_CCR, (float)PWM_MAX_CCR);
  Pid_SetIntegralLimit(&s_pid_spd_l, -SPEED_I_MAX_CCR, SPEED_I_MAX_CCR);

  Pid_Init(&s_pid_spd_r, SPEED_KP, SPEED_KI, SPEED_KD,
           -(float)PWM_MAX_CCR, (float)PWM_MAX_CCR);
  Pid_SetIntegralLimit(&s_pid_spd_r, -SPEED_I_MAX_CCR, SPEED_I_MAX_CCR);

  Pid_Init(&s_pid_dir, DIR_KP, DIR_KI, DIR_KD,
           -DIR_OUT_MAX_MMS, DIR_OUT_MAX_MMS);
  Pid_SetIntegralLimit(&s_pid_dir, -DIR_I_MAX_MMS, DIR_I_MAX_MMS);

  s_mode = CTRL_MODE_MANUAL_KNOB;
  s_auto_speed = AUTO_SPEED_MMS;
  s_lap = 0;
  s_heading_deg = 0.0f;
  s_lost_ms = 0U;
  s_armed = 1U;
  s_lap_done = 0U;
  s_hold = 0U;
  s_track_lost = 0U;
  s_telem_on = 0U;
  s_telem_at = 0U;
  s_autocal_done = 0U;
  reset_loops();

  Knob_SetPercent(0);
  BT_SetThrottlePercent(0);
}

CtrlMode_t Control_Mode(void)
{
  return s_mode;
}

const char *Control_ModeName(void)
{
  switch (s_mode)
  {
    case CTRL_MODE_AUTO:
      return "AUT";
    case CTRL_MODE_MANUAL_BT:
      return "BTH";
    case CTRL_MODE_MANUAL_KNOB:
    default:
      return "KNO";
  }
}

int32_t Control_LapCount(void)
{
  return s_lap;
}

float Control_HeadingDeg(void)
{
  return s_heading_deg;
}

float Control_AutoSpeed(void)
{
  return s_auto_speed;
}

uint8_t Control_TrackLost(void)
{
  return s_track_lost;
}

/* ------------------------------------------------------------------------- */
static void handle_bt_event(BT_Event_t ev)
{
  switch (ev)
  {
    case BT_EVT_AUTO:
      enter_auto();
      BT_SendStr("OK AUTO\r\n");
      break;

    case BT_EVT_MANUAL:
      enter_manual(CTRL_MODE_MANUAL_BT);
      BT_SendStr("OK MANUAL\r\n");
      break;

    case BT_EVT_STOP:
      s_hold = 1U;
      Motor_Stop();
      reset_loops();
      s_lap_done = 0U;
      BT_SendStr("OK STOP\r\n");
      break;

    case BT_EVT_CALIB:
      EC_Calibrate();
      BT_SendStr("OK CALIB\r\n");
      break;

    case BT_EVT_REPORT:
      Control_Telemetry();
      break;

    case BT_EVT_LAP_RESET:
      s_lap = 0;
      s_heading_deg = 0.0f;
      s_lap_done = 0U;
      BT_SendStr("OK LAP RESET\r\n");
      break;

    case BT_EVT_TELEM_TOGGLE:
      s_telem_on = (s_telem_on != 0U) ? 0U : 1U;
      s_telem_at = HAL_GetTick();
      BT_SendStr((s_telem_on != 0U) ? "OK STREAM ON\r\n" : "OK STREAM OFF\r\n");
      break;

    case BT_EVT_HELP:
      BT_SendStr("CMDS a=auto m=manual f=fwd b=back s=stop +/-=speed 0-9=level\r\n"
                 "     r=calib p=telemetry o=stream z=lap reset h=help\r\n");
      break;

    case BT_EVT_NONE:
    default:
      break;
  }
}

static void run_auto(uint32_t now)
{
  float err;

  if (s_armed == 0U)
  {
    if ((int32_t)(now - s_arm_at) >= 0)
    {
      s_armed = 1U;
      reset_loops();
    }
  }

  if (EC_OnTrack() != 0U)
  {
    s_lost_ms = 0U;
  }
  else
  {
    if (s_lost_ms < (EC_TRACK_LOST_MS * 4U))
    {
      s_lost_ms += CONTROL_PERIOD_MS;
    }
  }
  s_track_lost = (s_lost_ms >= EC_TRACK_LOST_MS) ? 1U : 0U;

  if ((s_armed == 0U) || (s_lap_done != 0U))
  {
    s_hold = 1U;
    return;
  }

#if (EC_LOST_ACTION_STOP != 0U)
  if (s_track_lost != 0U)
  {
    s_hold = 1U;
    return;
  }
#endif

  s_hold = 0U;

  /* outer loop: induction error -> steering speed difference.
     a positive error means the line sits left of the car, the car has to
     turn left, so the left wheel gets slower and the right one faster.   */
  err = EC_Error();
  s_steer = Pid_Update(&s_pid_dir, 0.0f, -err, CONTROL_DT_S);
  if (s_steer > DIR_OUT_MAX_MMS)
  {
    s_steer = DIR_OUT_MAX_MMS;
  }
  if (s_steer < -DIR_OUT_MAX_MMS)
  {
    s_steer = -DIR_OUT_MAX_MMS;
  }

  s_tgt_l = s_auto_speed - s_steer;
  s_tgt_r = s_auto_speed + s_steer;

  if (s_tgt_l > AUTO_SPEED_MAX)
  {
    s_tgt_l = AUTO_SPEED_MAX;
  }
  if (s_tgt_l < -AUTO_SPEED_MAX)
  {
    s_tgt_l = -AUTO_SPEED_MAX;
  }
  if (s_tgt_r > AUTO_SPEED_MAX)
  {
    s_tgt_r = AUTO_SPEED_MAX;
  }
  if (s_tgt_r < -AUTO_SPEED_MAX)
  {
    s_tgt_r = -AUTO_SPEED_MAX;
  }
}

static void run_manual(void)
{
  int16_t pct;

  s_hold = 0U;
  s_track_lost = 0U;

  pct = (s_mode == CTRL_MODE_MANUAL_KNOB) ? Knob_GetPercent() : BT_ThrottlePercent();

  if ((pct <= (int16_t)MAN_DEADBAND_PCT) && (pct >= -(int16_t)MAN_DEADBAND_PCT))
  {
    pct = 0;
  }

  s_steer = 0.0f;
  s_tgt_l = ((float)pct / 100.0f) * MAN_SPEED_MAX_MMS;
  s_tgt_r = s_tgt_l;
}

static void apply_speed_loop(void)
{
  float ff_l;
  float ff_r;

  if (s_hold != 0U)
  {
    Motor_Stop();
    reset_loops();
    return;
  }

  ff_l = MOTOR_FF_CCR_PER_MMS * s_tgt_l;
  ff_r = MOTOR_FF_CCR_PER_MMS * s_tgt_r;

  s_u_l = (int32_t)(Pid_Update(&s_pid_spd_l, s_tgt_l, Encoder_LeftMms(), CONTROL_DT_S) + ff_l);
  s_u_r = (int32_t)(Pid_Update(&s_pid_spd_r, s_tgt_r, Encoder_RightMms(), CONTROL_DT_S) + ff_r);

  /* Motor_SetSigned clamps to +/- PWM_MAX_CCR (the 58 % duty limit) */
  Motor_SetSigned(s_u_l, s_u_r);
}

static void update_lap(void)
{
  float omega_deg_s;
  float step;

  if ((s_mode != CTRL_MODE_AUTO) || (s_armed == 0U) || (s_lap_done != 0U))
  {
    return;
  }

  /* differential drive heading rate: omega = (v_right - v_left) / wheelbase */
  omega_deg_s = ((Encoder_RightMms() - Encoder_LeftMms()) / WHEEL_BASE_MM) * RAD_TO_DEG;
  step = omega_deg_s * CONTROL_DT_S;
  s_heading_deg += step;

#if (LAP_ENABLE != 0U)
  if (s_heading_deg >= LAP_ANGLE_DEG)
  {
    s_heading_deg -= LAP_ANGLE_DEG;
    s_lap++;
    if ((LAP_STOP_AFTER != 0U) && (s_lap >= LAP_TARGET_LAPS))
    {
      s_lap_done = 1U;
    }
  }
  else if (s_heading_deg <= -LAP_ANGLE_DEG)
  {
    s_heading_deg += LAP_ANGLE_DEG;
    s_lap++;
    if ((LAP_STOP_AFTER != 0U) && (s_lap >= LAP_TARGET_LAPS))
    {
      s_lap_done = 1U;
    }
  }
  else
  {
    /* still inside the current lap */
  }
#endif
}

void Control_Step(void)
{
  uint32_t now = HAL_GetTick();
  BT_Event_t ev;
  int8_t detents;

  Encoder_Update();
  EC_Update();
  BT_Poll();

  /* one offset calibration about 800 ms after power up: the car should be
     standing on the track at that moment, otherwise send 'r' with the car
     on the track */
  if ((s_autocal_done == 0U) && (now > 800U))
  {
    s_autocal_done = 1U;
    EC_Calibrate();
  }

  ev = BT_PopEvent();
  while (ev != BT_EVT_NONE)
  {
    handle_bt_event(ev);
    ev = BT_PopEvent();
  }

  /* turning the knob always takes over, even while the car is following */
  detents = Knob_PopDelta();
  if (detents != 0)
  {
    if (s_mode != CTRL_MODE_MANUAL_KNOB)
    {
      enter_manual(CTRL_MODE_MANUAL_KNOB);
      Knob_SetPercent(0);
    }
    Knob_AddPercent((int16_t)((int16_t)detents * (int16_t)KNOB_STEP_PCT));
  }

  if (s_mode == CTRL_MODE_AUTO)
  {
    run_auto(now);
  }
  else
  {
    run_manual();
  }

  apply_speed_loop();
  update_lap();

  if ((s_telem_on != 0U) && ((now - s_telem_at) >= TELEMETRY_PERIOD_MS))
  {
    s_telem_at = now;
    Control_Telemetry();
  }
}

void Control_Telemetry(void)
{
  char line[120];
  int32_t vl = (int32_t)Encoder_LeftMms();
  int32_t vr = (int32_t)Encoder_RightMms();
  int32_t tl = (int32_t)s_tgt_l;
  int32_t tr = (int32_t)s_tgt_r;

  (void)snprintf(line, sizeof(line),
                 "M=%s VL=%ld VR=%ld TL=%ld TR=%ld E=%ld D=%ld/%ld S=%u,%u,%u,%u LAP=%ld HD=%ld%s\r\n",
                 Control_ModeName(), (long)vl, (long)vr, (long)tl, (long)tr,
                 (long)(EC_Error() * 100.0f),
                 (long)Motor_LeftCommand(), (long)Motor_RightCommand(),
                 (unsigned)EC_Relative(0U), (unsigned)EC_Relative(1U),
                 (unsigned)EC_Relative(2U), (unsigned)EC_Relative(3U),
                 (long)s_lap, (long)s_heading_deg,
                 (s_hold != 0U) ? " HOLD" : "");
  BT_SendStr(line);
}

void Control_DrawHud(void)
{
  char line[24];
  int32_t avg = (int32_t)Encoder_AverageMms();

  OLED_Clear();

  /* row 0: mode, lap, track state */
  (void)snprintf(line, sizeof(line), "%s L%ld %s",
                 Control_ModeName(), (long)s_lap,
                 (s_track_lost != 0U) ? "LOST" : ((EC_OnTrack() != 0U) ? "TRK" : "---"));
  OLED_Str(0, 0, line, 1);

  /* row 1..2: the wheel speed in cm/s, drawn twice as large */
  if (avg < 0)
  {
    OLED_Char(0, 8, '-', 2);
    OLED_Int(12, 8, (avg <= -999) ? -999 : -avg / 10, 3, 2);
  }
  else
  {
    OLED_Int(0, 8, (avg > 999) ? 999 : avg / 10, 3, 2);
  }
  OLED_Str(42, 16, "CM/S", 1);

  /* row 3: target and induction error */
  (void)snprintf(line, sizeof(line), "T%4ld E%4ld",
                 (long)(int32_t)s_tgt_l, (long)(int32_t)(EC_Error() * 100.0f));
  OLED_Str(0, 24, line, 1);

  /* row 4: wheel speeds */
  (void)snprintf(line, sizeof(line), "L%4ld R%4ld",
                 (long)(int32_t)Encoder_LeftMms(), (long)(int32_t)Encoder_RightMms());
  OLED_Str(0, 32, line, 1);

  /* row 5: PWM commands (counts, 4200 = 100 %) */
  (void)snprintf(line, sizeof(line), "D%5ld/%5ld",
                 (long)Motor_LeftCommand(), (long)Motor_RightCommand());
  OLED_Str(0, 40, line, 1);

  /* row 6: the four induction channels above their baseline */
  (void)snprintf(line, sizeof(line), "S%4u%4u%4u%4u",
                 (unsigned)EC_Relative(0U), (unsigned)EC_Relative(1U),
                 (unsigned)EC_Relative(2U), (unsigned)EC_Relative(3U));
  OLED_Str(0, 48, line, 1);

  /* row 7: throttle sources and heading */
  (void)snprintf(line, sizeof(line), "K%4d B%4d H%3ld",
                 (int)Knob_GetPercent(), (int)BT_ThrottlePercent(),
                 (long)(int32_t)s_heading_deg);
  OLED_Str(0, 56, line, 1);
}
