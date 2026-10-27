/**
  ******************************************************************************
  * @file    CurrentControl.c
  * @brief   Simple d/q current closed loop (SimpleFOC 2.4 structure, C port).
  *
  * Clarke -> Park -> LPF -> PI (d and q) -> inverse Park + SVPWM
  * (setPhaseVoltage() already does the inverse Park and the SVPWM).
  ******************************************************************************
  */

#include "CurrentControl.h"
#include "BLDCMotor.h"
#include "foc_utils.h"
#include "pid.h"

/* ---- GB4310 motor + loop configuration -------------------------------- */
#define MOTOR_R_OHM         10.32f      /* phase resistance                */
#define MOTOR_L_H           0.00326f    /* phase inductance                */
#define CURRENT_BW_HZ       200.0f      /* current loop bandwidth          */

#define CURRENT_TS          0.0002f     /* 5 kHz: every 4th 20 kHz sample  */
#define CURRENT_ULIMIT      12.0f       /* V, vector magnitude limit       */
#define CURRENT_LIMIT       1.0f        /* A, iq reference limit           */
/* LPF time constant Tf = 1/(2*PI*bw*5); k = Ts/(Tf+Ts) */
#define CURRENT_LPF_TF      0.0001592f
#define CURRENT_LPF_K       (CURRENT_TS / (CURRENT_LPF_TF + CURRENT_TS))
/* P = L*2*PI*bw, I = R*2*PI*bw (SimpleFOC tuneCurrentController) */
#define CURRENT_KP          (MOTOR_L_H * 6.2831853f * CURRENT_BW_HZ)
#define CURRENT_KI          (MOTOR_R_OHM * 6.2831853f * CURRENT_BW_HZ)
/* ------------------------------------------------------------------------ */

volatile float current_d = 0.0f;
volatile float current_q = 0.0f;

static float iq_ref = 0.0f;
static uint8_t enabled = 0U;

/* Set by beep.c; the beeper drives the phases directly, stay out of it. */
extern volatile uint8_t beepPlaying;

typedef struct
{
  float k;
  float y;
} LPF_t;

static PID_t pid_d;
static PID_t pid_q;
static LPF_t lpf_d;
static LPF_t lpf_q;

static void LPF_Init(LPF_t *f)
{
  f->k = CURRENT_LPF_K;
  f->y = 0.0f;
}

static float LPF_Step(LPF_t *f, float x)
{
  f->y += f->k * (x - f->y);
  return f->y;
}

void CurrentControl_Init(void)
{
  PID_Init(&pid_d, CURRENT_KP, CURRENT_KI, 0.0f, 0.0f, CURRENT_ULIMIT);
  PID_Init(&pid_q, CURRENT_KP, CURRENT_KI, 0.0f, 0.0f, CURRENT_ULIMIT);
  LPF_Init(&lpf_d);
  LPF_Init(&lpf_q);

  iq_ref = 0.0f;
  current_d = 0.0f;
  current_q = 0.0f;
  enabled = 0U;
}

void CurrentControl_Enable(uint8_t enable)
{
  enabled = enable;

  if (enable == 0U)
  {
    PID_Reset(&pid_d);
    PID_Reset(&pid_q);
    LPF_Init(&lpf_d);
    LPF_Init(&lpf_q);
  }
}

void CurrentControl_SetTarget(float iq_target)
{
  iq_ref = _constrain(iq_target, -CURRENT_LIMIT, CURRENT_LIMIT);
}

void CurrentControl_Update(float ia, float ib, float theta_e)
{
  float s;
  float c;
  float i_alpha;
  float i_beta;
  float id;
  float iq;
  float ud;
  float uq;
  float umag;

  if ((enabled == 0U) || (beepPlaying != 0U))
  {
    return;
  }

  s = _sin(theta_e);
  c = _cos(theta_e);

  /* Clarke (ic implicit: ia + ib + ic = 0), two measured shunts */
  i_alpha = ia;
  i_beta  = _1_SQRT3 * ia + 2.0f * _1_SQRT3 * ib;

  /* Park */
  id =  i_alpha * c + i_beta * s;
  iq = -i_alpha * s + i_beta * c;

  /* Low-pass filter (cuts at 5x the loop bandwidth) */
  id = LPF_Step(&lpf_d, id);
  iq = LPF_Step(&lpf_q, iq);
  current_d = id;
  current_q = iq;

  /* d/q PI */
  ud = PID_Update(&pid_d, 0.0f - id, CURRENT_TS);
  uq = PID_Update(&pid_q, iq_ref - iq, CURRENT_TS);

  /* Limit the voltage vector (linear SVPWM limit is Vbus/sqrt(3) = 13.9 V) */
  umag = _sqrt(ud * ud + uq * uq);
  if (umag > CURRENT_ULIMIT)
  {
    float scale = CURRENT_ULIMIT / umag;
    ud *= scale;
    uq *= scale;
  }

  /* setPhaseVoltage(Uq, Ud, angle): inverse Park + SVPWM, PWM1 */
  setPhaseVoltage(uq, ud, theta_e);
}
