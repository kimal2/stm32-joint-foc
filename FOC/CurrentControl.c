/**
  ******************************************************************************
  * @file    CurrentControl.c
  * @brief   d/q 电流闭环控制（基于 SimpleFOC 2.4 结构的 C 语言移植）
  *
  * Clarke 变换 → Park 变换 → 低通滤波 → d/q 轴 PI → 逆 Park 变换与 SVPWM
  * （setPhaseVoltage() 内部已实现逆 Park 变换和 SVPWM）。
  ******************************************************************************
  */

#include "CurrentControl.h"
#include "CurrentControlConfig.h"
#include "BLDCMotor.h"
#include "foc_utils.h"
#include "pid.h"
#include "stm32f1xx.h"

volatile float current_d = 0.0f;
volatile float current_q = 0.0f;

/* 最近一次有效电流环计算的 DWT 原始周期差值，供 Ozone 观察。 */
volatile uint32_t current_loop_cycles = 0U;

static float iq_ref = 0.0f;
static uint8_t enabled = 0U;

/* 由 beep.c 设置；蜂鸣时直接驱动电机各相，电流环暂停输出。 */
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
  /* 启用 DWT 周期计数，不清零计数器，避免影响调试器的其他计时。 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();
  current_loop_cycles = 0U;

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
  uint32_t cycle_start;

  if ((enabled == 0U) || (beepPlaying != 0U))
  {
    return;
  }

  /* 从三角函数计算开始计数；未使能或蜂鸣时不更新测量结果。 */
  cycle_start = DWT->CYCCNT;

  s = _sin(theta_e);
  c = _cos(theta_e);

  /* Clarke 变换：使用两相实测电流，隐含 ic = -(ia + ib) */
  i_alpha = ia;
  i_beta  = _1_SQRT3 * ia + 2.0f * _1_SQRT3 * ib;

  /* Park 变换 */
  id =  i_alpha * c + i_beta * s;
  iq = -i_alpha * s + i_beta * c;

  /* 低通滤波（截止频率为电流环带宽的 5 倍） */
  id = LPF_Step(&lpf_d, id);
  iq = LPF_Step(&lpf_q, iq);
  current_d = id;
  current_q = iq;

  /* d/q 轴 PI 控制 */
  ud = PID_Update(&pid_d, 0.0f - id, CURRENT_TS);
  uq = PID_Update(&pid_q, iq_ref - iq, CURRENT_TS);

  /* 限制电压矢量幅值（SVPWM 线性区上限为 Vbus/sqrt(3) = 13.9 V） */
  umag = _sqrt(ud * ud + uq * uq);
  if (umag > CURRENT_ULIMIT)
  {
    float scale = CURRENT_ULIMIT / umag;
    ud *= scale;
    uq *= scale;
  }

  /* setPhaseVoltage(Uq, Ud, angle)：逆 Park 变换、SVPWM 和 PWM1 输出 */
  setPhaseVoltage(uq, ud, theta_e);

  /* 包含坐标变换、滤波、PI、限幅及 PWM 更新；无符号相减兼容计数回绕。 */
  current_loop_cycles = DWT->CYCCNT - cycle_start;
}
