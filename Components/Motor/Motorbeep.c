/**
  ******************************************************************************
  * @file    Motorbeep.c
  * @brief   利用电机绕组实现蜂鸣，移植自示例项目
  ******************************************************************************
  * TIM4 配置为 1 MHz 时基（72 MHz / 72）。
  * ARR = period_us / 2，更新中断每半个周期切换一次电压矢量，
  * 从而使绕组电流以 1 / period_us 对应的频率振荡。
  *
  * 蜂鸣输出前检查目标电压是否为 0，
  * 以免与正常 FOC 输出冲突。
  ******************************************************************************
  */

#include "Motorbeep.h"
#include "tim.h"
#include "foc_utils.h"
#include "FOC.h"

/* 由 FOC.c 的 loopFOC() 及电流环读取 */
volatile uint8_t beepPlaying = 0;

/* 定义在 APP/MotorApp.c 中 */
extern volatile float motor_target_voltage;

void Beep_Init(void)
{
  /* 72 MHz / (71 + 1) = 1 MHz，定时器每计数一次对应 1 微秒 */
  __HAL_TIM_SET_PRESCALER(&htim4, 71U);

  HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void Beep_Play(uint16_t period_us)
{
  if (period_us != 0U)
  {
    __HAL_TIM_SET_AUTORELOAD(&htim4, (uint32_t)(period_us / 2U));
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    HAL_TIM_Base_Start_IT(&htim4);
    beepPlaying = 1U;
  }
  else
  {
    HAL_TIM_Base_Stop_IT(&htim4);
    beepPlaying = 0U;
  }
}

void Beep_PlayNotes(uint8_t num, const uint16_t notes[][2])
{
  for (uint8_t i = 0; i < num; i++)
  {
    Beep_Play(notes[i][0]);
    HAL_Delay(notes[i][1]);
  }
  Beep_Play(0);
}

/*
 * TIM4 更新回调：由生成的 TIM4_IRQHandler 经 HAL_TIM_IRQHandler 调用。
 * 电压矢量在 0 度和 60 度之间交替切换，
 * 使绕组电流产生可听声音，
 * 以期不产生净转矩。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t flip = 0;

  if (htim->Instance != TIM4)
  {
    return;
  }

  if (motor_target_voltage == 0.0f)
  {
    setPhaseVoltage(6, 0.0f,
                    (float)(_PI_3 * (float)flip));
  }
  flip = (uint8_t)!flip;
}
