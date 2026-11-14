#include "MotorApp.h"
#include "MotorAppConfig.h"
#include "CurrentControlConfig.h"
#include "CurrentControl.h"
#include "FOC.h"
#include "MagneticSensor.h"
#include "as5600.h"
#include "Motorbeep.h"
#include "adc.h"
#include "tim.h"
#include <math.h>

/* 采样状态开始 */
volatile float motor_target_voltage = 1.0f;   /* 单位 V，用于电压模式及蜂鸣门控 */
volatile float motor_target_current = 0.0f;   /* 单位 A，记录已应用的 q 轴目标电流 */

/* ADC1 注入组采样结果：序列 1 = CH5（U7 / R9），序列 2 = CH6（U10 / R12） */
volatile uint16_t adc_raw_a = 0U;
volatile uint16_t adc_raw_b = 0U;
volatile float phase_current_a = 0.0f;
volatile float phase_current_b = 0.0f;
volatile float phase_current_c = 0.0f;
volatile uint32_t adc_sample_count = 0U;

/* 零点单位为 ADC 计数，可带小数；上电重新采集，不写入 Flash。 */
volatile float current_offset_adc_a = 0.0f;
volatile float current_offset_adc_b = 0.0f;
/* 0 = 未校准，1 = 采集中，2 = 校准成功，3 = 校准失败。 */
volatile uint8_t current_offset_status = 0U;
volatile uint32_t current_offset_sample_count = 0U;
static volatile uint32_t current_offset_sum_a = 0U;
static volatile uint32_t current_offset_sum_b = 0U;

static uint8_t current_loop_div = 0U;
/* 采样状态结束 */

volatile MotorAppState motor_app_state = MOTOR_APP_INITIALIZING;
volatile uint32_t motor_command_elapsed_ms = 0U;  /* SysTick 每 1 ms 加一，有效命令应用后清零。 */
volatile uint32_t motor_command_count = 0U;
volatile float motor_velocity_rad_s = 0.0f;
static uint32_t velocity_tick;

static HAL_StatusTypeDef CurrentSense_CalibrateOffset(void)
{
  uint32_t start_tick;
  float offset_a;
  float offset_b;
  const float min_adc = CURRENT_OFFSET_MIN_V * (ADC_FULL_SCALE / ADC_VREF_V);
  const float max_adc = CURRENT_OFFSET_MAX_V * (ADC_FULL_SCALE / ADC_VREF_V);

  /* 必须在电机静止、没有相电流时执行，驱动保持关闭直到校准通过。 */
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
  CurrentControl_Enable(0U);
  current_offset_status = 0U;
  HAL_Delay(CURRENT_OFFSET_SETTLE_MS);
  current_offset_sum_a = 0U;
  current_offset_sum_b = 0U;
  current_offset_sample_count = 0U;
  start_tick = HAL_GetTick();
  current_offset_status = 1U;

  /* 只累加新的注入组采样；达到指定次数后，中断不再修改累加结果。 */
  while (current_offset_sample_count < CURRENT_OFFSET_SAMPLES)
  {
    if ((HAL_GetTick() - start_tick) >= CURRENT_OFFSET_TIMEOUT_MS)
    {
      current_offset_status = 3U;
      return HAL_TIMEOUT;
    }
  }

  offset_a = (float)current_offset_sum_a / (float)CURRENT_OFFSET_SAMPLES;
  offset_b = (float)current_offset_sum_b / (float)CURRENT_OFFSET_SAMPLES;
  if ((offset_a < min_adc) || (offset_a > max_adc) ||
      (offset_b < min_adc) || (offset_b > max_adc))
  {
    current_offset_status = 3U;
    return HAL_ERROR;
  }

  current_offset_adc_a = offset_a;
  current_offset_adc_b = offset_b;
  current_loop_div = 0U;
  current_offset_status = 2U;
  return HAL_OK;
}

HAL_StatusTypeDef MotorApp_Init(void)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;

  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
  motor_app_state = MOTOR_APP_INITIALIZING;
  motor_target_current = 0.0f;
  CurrentControl_Init();

  /*
   * 使能 CCR 预装载，在定时器更新事件发生时统一更新占空比。
   * 配合中心对齐 PWM，可避免 FOC 在周期中途修改 CCR 时
   * 产生不对称的脉冲。
   */
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);

  /* 使能驱动器前先输出零电压矢量 */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period / 2U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period / 2U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, period / 2U);
  htim2.Instance->EGR = TIM_EGR_UG;   /* 将预装载的 CCR 值更新到有效寄存器 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  /*
   * CH4 不输出到引脚，仅生成 OC4REF 信号，并将其路由到
   * TIM2 TRGO，每个 PWM 周期触发一次 ADC1 注入组采样。
   */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* 电流采样：先执行一次 ADC 校准，再由 TIM2 TRGO 触发注入组转换 */
  if ((HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) ||
      (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK))
  {
    current_offset_status = 3U;
    MotorApp_Disable(MOTOR_APP_FAULT);
    return HAL_ERROR;
  }

  if (CurrentSense_CalibrateOffset() != HAL_OK)
  {
    MotorApp_Disable(MOTOR_APP_FAULT);
    return HAL_ERROR;
  }

  /* 通过已验证的硬件 I2C1 驱动读取 AS5600 */
  MagneticSensor_Init();

  /* 本板 FOC 参数：24 V 母线、4310 电机 */
  voltage_power_supply = 24.0f;
  voltage_limit = 4.0f;
  voltage_sensor_align = 4.0f;

  torque_controller = Type_foc_current;

  Motor_init();
  Motor_initFOC();

  /* 传感器校准会直接驱动电机各相，因此校准结束后才使能电流环。 */
  electrical_angle = electricalAngle();
  if ((as5600_error != 0U) || ((sensor_direction != CW) && (sensor_direction != CCW)) ||
      (pole_pairs <= 0) || !isfinite(zero_electric_angle))
  {
    MotorApp_Disable(MOTOR_APP_FAULT);
    return HAL_ERROR;
  }
  CurrentControl_SetTarget(0.0f);
  CurrentControl_Enable(1U);
  motor_command_elapsed_ms = 0U;
  motor_app_state = MOTOR_APP_RUNNING;
  return HAL_OK;
}

void MotorApp_OnAdcSample(uint16_t raw_a, uint16_t raw_b)
{
  adc_raw_a = raw_a;
  adc_raw_b = raw_b;

  adc_sample_count++;

  if (current_offset_status == 1U)
  {
    if (current_offset_sample_count < CURRENT_OFFSET_SAMPLES)
    {
      current_offset_sum_a += adc_raw_a;
      current_offset_sum_b += adc_raw_b;
      current_offset_sample_count++;
    }
  }
  else if (current_offset_status == 2U)
  {
    phase_current_a = CURRENT_SENSE_SIGN * ((float)adc_raw_a - current_offset_adc_a)
                    * (ADC_VREF_V / ADC_FULL_SCALE / CURRENT_V_PER_A);
    phase_current_b = CURRENT_SENSE_SIGN * ((float)adc_raw_b - current_offset_adc_b)
                    * (ADC_VREF_V / ADC_FULL_SCALE / CURRENT_V_PER_A);

    /* 两路实测电流重构 C 相：Ia + Ib + Ic = 0。 */
    phase_current_c = -(phase_current_a + phase_current_b);

    /* 每 2 次约 20 kHz 采样执行一次，电流环约 10 kHz。 */
    if (++current_loop_div >= CURRENT_ADC_DECIMATION)
    {
      current_loop_div = 0U;
      CurrentControl_Update(phase_current_a, phase_current_b, electrical_angle);
    }
  }

}

void MotorApp_Disable(MotorAppState reason)
{
  /* 先关闭功率输出，再与 ADC 中断互斥地清理控制器状态。 */
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  CurrentControl_Enable(0U);
  CurrentControl_SetTarget(0.0f);
  motor_target_current = 0.0f;
  motor_app_state = reason;
  __set_PRIMASK(irq_state);
  Beep_Play(0U);
  setPhaseVoltage(0.0f, 0.0f, electrical_angle);
}

HAL_StatusTypeDef MotorApp_Enable(void)
{
  if (((motor_app_state != MOTOR_APP_DISABLED) && (motor_app_state != MOTOR_APP_TIMEOUT)) ||
      (current_offset_status != 2U) || (as5600_error != 0U))
  {
    return HAL_ERROR;
  }

  /* 恢复采用零目标，不能执行超时前残留的指令。 */
  setPhaseVoltage(0.0f, 0.0f, electrical_angle);
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  CurrentControl_SetTarget(0.0f);
  CurrentControl_Enable(1U);
  motor_target_current = 0.0f;
  motor_command_elapsed_ms = 0U;
  motor_app_state = MOTOR_APP_RUNNING;
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
  __set_PRIMASK(irq_state);
  return HAL_OK;
}

HAL_StatusTypeDef MotorApp_ApplyCurrent(float current_a, uint32_t received_tick)
{
  /* 使用中断接收时间，防止主循环延迟后把过期缓存误当作新命令。 */
  if ((uint32_t)(HAL_GetTick() - received_tick) >= MOTOR_COMMAND_TIMEOUT_MS) return HAL_TIMEOUT;

  if (current_a > CURRENT_LIMIT) current_a = CURRENT_LIMIT;
  if (current_a < -CURRENT_LIMIT) current_a = -CURRENT_LIMIT;

  /* 状态检查、应用目标和计数清零必须连续完成，避免被 SysTick 超时关断打断。 */
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  if ((motor_app_state != MOTOR_APP_RUNNING) && (motor_app_state != MOTOR_APP_TIMEOUT))
  {
    __set_PRIMASK(irq_state);
    return HAL_ERROR;
  }
  uint8_t resume_from_timeout = (motor_app_state == MOTOR_APP_TIMEOUT);
  motor_target_current = current_a;
  CurrentControl_SetTarget(current_a);
  motor_command_elapsed_ms = 0U;
  motor_app_state = MOTOR_APP_RUNNING;
  if (resume_from_timeout != 0U)
  {
    /* 超时只关闭了驱动引脚；先写入本次新目标，再恢复使能，不沿用旧指令。 */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
  }
  motor_command_count++;
  __set_PRIMASK(irq_state);
  return HAL_OK;
}

void MotorApp_OnSysTick1ms(void)
{
  /* 仅在运行期间计时；超时后等待新的合法电流指令恢复使能。 */
  if (motor_app_state != MOTOR_APP_RUNNING) return;

  if (++motor_command_elapsed_ms >= MOTOR_COMMAND_TIMEOUT_MS)
  {
    /* 超时路径只关驱动、清零目标并记录状态，不清 PI、不改 PWM、不处理蜂鸣器。 */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
    CurrentControl_SetTarget(0.0f);
    motor_target_current = 0.0f;
    motor_app_state = MOTOR_APP_TIMEOUT;
  }
}

void MotorApp_Update(void)
{
  uint32_t now = HAL_GetTick();
  /* 阻塞 I2C 读取留在主循环，不放到 SysTick 或电流中断中。 */
  loopFOC();
  if ((uint32_t)(now - velocity_tick) >= 1U)
  {
    velocity_tick = now;
    motor_velocity_rad_s = getVelocity();
  }
}
