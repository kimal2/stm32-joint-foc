/**
 * @file CurrentControlIQ.c
 * @brief IQmath Q20 电流环：Clarke、Park、低通滤波、PI、矢量限幅及 SVPWM。
 * 输入与调试输出保留浮点接口，环内计算及 PWM 占空比使用定点数。
 */
#include "CurrentControl.h"
#include "CurrentControlConfig.h"
#include "FOC.h"
#include "IQmathLib.h"
#include "stm32f1xx.h"
#include "tim.h"

_Static_assert(sizeof(_iq20) == 4, "IQmath requires 32-bit long");

/* Q20 范围 [-2048, 2048)，12 V 两轴平方和最大为 288，不会溢出。
 * 仅对输入施加 ±16 A 的数值保护；本板 ADC 实际范围约为 ±1.65 A。
 * Ki 约为 12968，不能直接转 Q20；先预计算 Ki*Ts/2，约为 0.6484。
 */
#define IQ_CURRENT_INPUT_LIMIT 16.0f
#define IQ_INV_SQRT3           _IQ20(0.57735026919f)
#define IQ_SQRT3_2             _IQ20(0.86602540378f)
#define IQ_KP                  _IQ20(CURRENT_KP)
#define IQ_KI_TS_HALF          _IQ20(CURRENT_KI * CURRENT_TS * 0.5f)
#define IQ_LPF_K               _IQ20(CURRENT_LPF_K)
#define IQ_ULIMIT              _IQ20(CURRENT_ULIMIT)
#define IQ_ULIMIT_SQUARED      _IQ20(CURRENT_ULIMIT * CURRENT_ULIMIT)

volatile float current_d = 0.0f;
volatile float current_q = 0.0f;
volatile uint32_t current_loop_cycles = 0U;

extern volatile uint8_t beepPlaying;

typedef struct
{
  _iq20 integral;
  _iq20 error_prev;
} CurrentPI_IQ;

static CurrentPI_IQ pi_d;
static CurrentPI_IQ pi_q;
static _iq20 lpf_d;
static _iq20 lpf_q;
static volatile _iq20 iq_ref;
static volatile uint8_t enabled;
static float bus_voltage_cached;
static _iq20 bus_voltage_iq;
static _iq20 bus_reciprocal_iq;

static _iq20 ClampIQ(_iq20 value, _iq20 limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static _iq20 CurrentToIQ(float value, float limit)
{
  if (value != value) return 0;  /* 避免 NaN 转整数 */
  if (value > limit) value = limit;
  if (value < -limit) value = -limit;
  return _IQ20(value);
}

static void ResetState(void)
{
  pi_d.integral = 0;
  pi_d.error_prev = 0;
  pi_q.integral = 0;
  pi_q.error_prev = 0;
  lpf_d = 0;
  lpf_q = 0;
}

static _iq20 CurrentPI_Update(CurrentPI_IQ *pi, _iq20 error)
{
  /* 保留原有梯形积分，以及积分、输出分别限幅的规则；电流环 D 原本为 0。 */
  pi->integral = ClampIQ(pi->integral +
      _IQ20mpy(IQ_KI_TS_HALF, error + pi->error_prev), IQ_ULIMIT);
  pi->error_prev = error;
  return ClampIQ(_IQ20mpy(IQ_KP, error) + pi->integral, IQ_ULIMIT);
}

static void UpdateBusVoltage(void)
{
  if (voltage_power_supply == bus_voltage_cached) return;

  bus_voltage_cached = voltage_power_supply;
  /* 防止无效母线值引发除零或 Q20 转换溢出；正常配置为 24 V。 */
  if ((voltage_power_supply >= 1.0f) && (voltage_power_supply <= 512.0f))
  {
    bus_voltage_iq = _IQ20(voltage_power_supply);
    bus_reciprocal_iq = _IQ20div(_IQ20(1.0f), bus_voltage_iq);
  }
  else
  {
    bus_voltage_iq = 0;
    bus_reciprocal_iq = 0;
  }
}

static uint32_t PhaseToCCR(_iq20 phase, uint32_t period)
{
  _iq20 duty;
  if ((phase <= 0) || (bus_voltage_iq == 0)) return 0U;
  if (phase >= bus_voltage_iq) return period;

  duty = _IQ20mpy(phase, bus_reciprocal_iq);
  /* 64 位中间积避免占空比乘 PWM 周期时溢出，结果向下取整。 */
  return (uint32_t)(((uint64_t)duty * period) >> 20);
}

static void SetPhaseVoltageIQ(_iq20 uq, _iq20 ud, _iq20 s, _iq20 c)
{
  _iq20 alpha = _IQ20mpy(c, ud) - _IQ20mpy(s, uq);
  _iq20 beta = _IQ20mpy(s, ud) + _IQ20mpy(c, uq);
  _iq20 ua = alpha;
  _iq20 ub = -(alpha / 2) + _IQ20mpy(IQ_SQRT3_2, beta);
  _iq20 uc = -(alpha / 2) - _IQ20mpy(IQ_SQRT3_2, beta);
  _iq20 umin = (ua < ub) ? ua : ub;
  _iq20 umax = (ua > ub) ? ua : ub;
  _iq20 center;
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;

  if (uc < umin) umin = uc;
  if (uc > umax) umax = uc;
  center = bus_voltage_iq / 2 - (umax + umin) / 2;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PhaseToCCR(ua + center, period));
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PhaseToCCR(ub + center, period));
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PhaseToCCR(uc + center, period));
}

void CurrentControl_Init(void)
{
  enabled = 0U;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();
  current_loop_cycles = 0U;
  current_d = 0.0f;
  current_q = 0.0f;
  iq_ref = 0;
  ResetState();
  bus_voltage_cached = -1.0f;
  UpdateBusVoltage();
}

void CurrentControl_Enable(uint8_t enable)
{
  enabled = enable;
  if (enable == 0U) ResetState();
}

void CurrentControl_SetTarget(float iq_target)
{
  iq_ref = CurrentToIQ(iq_target, CURRENT_LIMIT);
}

void CurrentControl_Update(float ia, float ib, float theta_e)
{
  _iq20 ia_iq;
  _iq20 ib_iq;
  _iq20 theta;
  _iq20 s;
  _iq20 c;
  _iq20 alpha;
  _iq20 beta;
  _iq20 id;
  _iq20 iq;
  _iq20 ud;
  _iq20 uq;
  _iq20 magnitude_squared;
  uint32_t cycle_start;

  if ((enabled == 0U) || (beepPlaying != 0U)) return;

  /* 计入输入格式转换、全部环内计算、调试输出转换及 PWM 更新。 */
  cycle_start = DWT->CYCCNT;
  ia_iq = CurrentToIQ(ia, IQ_CURRENT_INPUT_LIMIT);
  ib_iq = CurrentToIQ(ib, IQ_CURRENT_INPUT_LIMIT);
  /* 调用者的 electrical_angle 已归一化到 [0, 2PI)，单位为 rad。 */
  theta = _IQ20(theta_e);
  s = _IQ20sin(theta);
  c = _IQ20cos(theta);

  /* Clarke 变换：两相实测电流转换为静止坐标系 alpha/beta 电流。 */
  alpha = ia_iq;
  beta = _IQ20mpy(IQ_INV_SQRT3, ia_iq) +
         _IQ20mpy(2 * IQ_INV_SQRT3, ib_iq);
  /* Park 变换：用转子电角度得到 d/q 电流，再滤波后送入 PI。 */
  id = _IQ20mpy(alpha, c) + _IQ20mpy(beta, s);
  iq = -_IQ20mpy(alpha, s) + _IQ20mpy(beta, c);

  lpf_d += _IQ20mpy(IQ_LPF_K, id - lpf_d);
  lpf_q += _IQ20mpy(IQ_LPF_K, iq - lpf_q);
  current_d = _IQ20toF(lpf_d);
  current_q = _IQ20toF(lpf_q);

  /* d 轴目标为 0，q 轴目标来自 CAN 指令；PI 输出为 d/q 轴电压。 */
  ud = CurrentPI_Update(&pi_d, -lpf_d);
  uq = CurrentPI_Update(&pi_q, iq_ref - lpf_q);

  /* 两轴合成电压超限时等比例缩小，保持电压矢量方向。 */
  magnitude_squared = _IQ20mpy(ud, ud) + _IQ20mpy(uq, uq);
  if (magnitude_squared > IQ_ULIMIT_SQUARED)
  {
    _iq20 magnitude = _IQ20sqrt(magnitude_squared);
    _iq20 scale = _IQ20div(IQ_ULIMIT, magnitude);
    ud = _IQ20mpy(ud, scale);
    uq = _IQ20mpy(uq, scale);
  }

  UpdateBusVoltage();
  /* 复用 Park 变换的正余弦，直接以定点 SVPWM 更新 CCR。 */
  SetPhaseVoltageIQ(uq, ud, s, c);
  current_loop_cycles = DWT->CYCCNT - cycle_start;
}
