/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "MagneticSensor.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "CurrentControl.h"
#include "SpeedControl.h"
#include "PositionControl.h"
#include "Commander.h"
#include "Vofa.h"
#include "beep.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Current sense: INA199A1 (gain 50 V/V) on a 20 mOhm shunt,
 * REF = 1.65 V (R8/R10 divider), ADC 12 bit with VDDA = 3.3 V.
 *   Vout = 1.65 V + 50 * 0.020 * I   ->   1 V per A
 * With IN+ = GND and IN- = PGND, Vout drops while the low side conducts,
 * so (Vout - 1.65 V) is already the phase current flowing into the motor.
 */
#define ADC_VREF_V            3.3f
#define ADC_FULL_SCALE        4096.0f
#define CURRENT_OFFSET_V      1.65f
#define CURRENT_V_PER_A       1.0f      /* 50 V/V * 0.020 ohm */
#define CURRENT_SENSE_SIGN    (1.0f)

/* 1 = d/q current closed loop (torque in A), 0 = voltage mode (torque in V) */
#define USE_CURRENT_LOOP      1

/* 1 = outer speed loop (target in rad/s, PI output = iq reference).
   Requires USE_CURRENT_LOOP == 1. */
#define USE_SPEED_LOOP        1

#if USE_SPEED_LOOP && !USE_CURRENT_LOOP
#error "USE_SPEED_LOOP requires USE_CURRENT_LOOP"
#endif

/* 1 = outer position loop (target in rad, output = speed reference).
   Requires USE_SPEED_LOOP == 1. */
#define USE_POSITION_LOOP     1

#if USE_POSITION_LOOP && !USE_SPEED_LOOP
#error "USE_POSITION_LOOP requires USE_SPEED_LOOP"
#endif

/* VOFA+ JustFloat telemetry on USART1 (115200 8N1) */
#define VOFA_SEND_CH          3U    /* floats per frame, <= VOFA_MAX_CH */
#define PRINT_PERIOD_MS       10U   /* frame period in ms               */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float motor_target_voltage = 1.0f;   /* V, voltage mode only */
volatile float motor_target_current = 0.2f;   /* A, direct torque mode */
volatile float motor_target_speed = 10.0f;    /* rad/s (~95 rpm), speed mode */
volatile float motor_target_position = 0.0f;  /* rad (multi-turn), position mode */

/* ADC1 injected results: rank1 = CH5 (U7 / R9), rank2 = CH6 (U10 / R12) */
volatile uint16_t adc_raw_a = 0U;
volatile uint16_t adc_raw_b = 0U;
volatile float phase_current_a = 0.0f;
volatile float phase_current_b = 0.0f;
volatile float phase_current_c = 0.0f;
volatile uint32_t adc_sample_count = 0U;

static uint8_t current_loop_div = 0U;
static uint32_t speed_tick = 0U;

uint32_t last_print_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SimpleFOC_Init(void)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;

  /*
   * Enable CCR preload so duty updates are latched at the timer update event.
   * With center-aligned PWM this avoids asymmetric pulses when the FOC loop
   * writes a new CCR in the middle of a period.
   */
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);

  /* Zero voltage vector before enabling the driver */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period / 2U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period / 2U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, period / 2U);
  htim2.Instance->EGR = TIM_EGR_UG;   /* latch the preloaded CCR values */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  /*
   * CH4 has no pin: it only generates OC4REF, which is routed to
   * TIM2 TRGO and triggers the ADC1 injected group once per PWM period.
   */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* Current sensing: calibrate once, then let TIM2 TRGO run the injected group */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  /* AS5600 through the verified hardware I2C1 driver */
  MagneticSensor_Init();

  /* FOC parameters for this board: 24V bus, 4310 motor */
  voltage_power_supply = 24.0f;
  voltage_limit = 4.0f;
  voltage_sensor_align = 4.0f;

  CurrentControl_Init();
  SpeedControl_Init();
  PositionControl_Init();
#if USE_POSITION_LOOP
  controller = Type_angle;
#elif USE_SPEED_LOOP
  controller = Type_velocity;
#else
  controller = Type_torque;
#endif
#if USE_CURRENT_LOOP
  torque_controller = Type_foc_current;
#else
  torque_controller = Type_voltage;
#endif

  Motor_init();
  Motor_initFOC();

#if USE_CURRENT_LOOP
  /* Enable only after the sensor alignment, which drives the phases directly. */
  CurrentControl_SetTarget(0.0f);
  CurrentControl_Enable(1U);
#endif
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  SimpleFOC_Init();
  Commander_Init();
  speed_tick = HAL_GetTick();

#if USE_POSITION_LOOP
  /* Hold the position we are at after alignment instead of jumping to 0 */
  motor_target_position = shaft_angle;
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Commander_Poll();
    loopFOC();

#if USE_SPEED_LOOP
    /* Outer speed loop at ~1 kHz: PI output is the iq reference */
    {
      uint32_t now = HAL_GetTick();
      if (now != speed_tick)
      {
        float dt = (float)(now - speed_tick) * 0.001f;
        speed_tick = now;

        shaft_velocity = getVelocity();

#if USE_POSITION_LOOP
        /* position P -> speed reference (limited inside PositionControl) */
        PositionControl_SetTarget(motor_target_position);
        SpeedControl_SetTarget(PositionControl_Update(shaft_angle, dt));
#else
        SpeedControl_SetTarget(motor_target_speed);
#endif
        CurrentControl_SetTarget(SpeedControl_Update(shaft_velocity, dt));
      }
    }
#else
    CurrentControl_SetTarget(motor_target_current);
#endif
    setTargetVotage(motor_target_voltage);

    /* VOFA+ JustFloat telemetry: fill vofa_data[] with the channels you want */
    if ((HAL_GetTick() - last_print_tick) >= PRINT_PERIOD_MS)
    {
      float vofa_data[VOFA_SEND_CH];
      last_print_tick = HAL_GetTick();

      vofa_data[0] = phase_current_a;
      vofa_data[1] = phase_current_b;
      vofa_data[2] = phase_current_c;

      VOFA_Send(vofa_data, VOFA_SEND_CH);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
 * Runs at the PWM rate (20 kHz), once per TIM2 TRGO.
 * The trigger sits right before the counter peak, i.e. inside the window
 * where all three low-side FETs are on, so the PGND shunts carry current.
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  float volts_a;
  float volts_b;

  if (hadc->Instance != ADC1)
  {
    return;
  }

  adc_raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
  adc_raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);

  volts_a = (float)adc_raw_a * (ADC_VREF_V / ADC_FULL_SCALE);
  volts_b = (float)adc_raw_b * (ADC_VREF_V / ADC_FULL_SCALE);

  phase_current_a = CURRENT_SENSE_SIGN * (volts_a - CURRENT_OFFSET_V) / CURRENT_V_PER_A;
  phase_current_b = CURRENT_SENSE_SIGN * (volts_b - CURRENT_OFFSET_V) / CURRENT_V_PER_A;

  /*
   * Only two shunts are populated (U7 on PGND1, U10 on PGND2), so phase C is
   * reconstructed from Kirchhoff's current law: Ia + Ib + Ic = 0.
   * Keep the same sign convention as the two measured phases.
   */
  phase_current_c = -(phase_current_a + phase_current_b);

  adc_sample_count++;

  /* d/q current loop at 5 kHz (every 4th 20 kHz sample), synchronized to PWM */
  if (++current_loop_div >= 4U)
  {
    current_loop_div = 0U;
    CurrentControl_Update(phase_current_a, phase_current_b, electrical_angle);
  }

  /*
   * STM32F1 HAL quirk (stm32f1xx_hal_adc.c, HAL_ADC_IRQHandler): when the
   * regular group is configured for software start + single conversion
   * (which is the case here, since the regular group is disabled), the HAL
   * disables the JEOC interrupt after the first injected conversion.
   * The hardware itself keeps converting on every TIM2 TRGO, so just re-arm
   * the interrupt. Clearing JEOC first avoids a spurious re-entry.
   */
  __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);
  __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
