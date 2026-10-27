/**
  ******************************************************************************
  * @file    beep.c
  * @brief   "Beeper" using the motor windings, ported from the sample project
  ******************************************************************************
  * TIM4 is configured as a 1 MHz time base (72 MHz / 72).
  * ARR = period_us / 2, so the update interrupt toggles the voltage vector
  * every half period; the winding current then vibrates at 1 / period_us.
  *
  * The sample checks "target voltage == 0" before beeping so that the beep
  * does not fight the normal FOC output.
  ******************************************************************************
  */

#include "beep.h"
#include "tim.h"
#include "foc_utils.h"
#include "BLDCMotor.h"

/* Referenced by loopFOC() in BLDCMotor.c */
volatile uint8_t beepPlaying = 0;

/* Defined in main.c */
extern volatile float motor_target_voltage;

void Beep_Init(void)
{
  /* 72 MHz / (71 + 1) = 1 MHz, one timer tick = 1 us */
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
 * TIM4 update callback (called from the generated TIM4_IRQHandler via
 * HAL_TIM_IRQHandler). The vector alternates between 0 deg and 60 deg,
 * which is enough to make the winding current audible without producing
 * net torque.
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
