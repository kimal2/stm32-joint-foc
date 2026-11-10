/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MagneticSensor.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "CurrentControl.h"
#include "CurrentControlConfig.h"
#include "beep.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * 电流采样：INA199A1（增益 50 V/V），采样电阻为 20 毫欧，
 * 参考电压为 1.65 V（R8/R10 分压），ADC 为 12 位，VDDA = 3.3 V。
 *   Vout = 1.65 V + 50 * 0.020 * I，即每安培电流对应 1 V 电压变化。
 * IN+ 接 GND、IN- 接 PGND，低侧导通时输出电压下降，
 * 开机实测两路零电流 ADC 均值，后续减去各自零点后换算相电流。
 */
#define ADC_VREF_V            3.3f
#define ADC_FULL_SCALE        4096.0f
#define CURRENT_V_PER_A       1.0f      /* 50 V/V * 0.020 欧姆 */
#define CURRENT_SENSE_SIGN    (1.0f)
#define CURRENT_OFFSET_SAMPLES       1024U
#define CURRENT_OFFSET_SETTLE_MS     20U
#define CURRENT_OFFSET_TIMEOUT_MS    500U
#define CURRENT_OFFSET_MIN_V         1.40f  /* 标称 1.65 V，允许 ±0.25 V 偏差 */
#define CURRENT_OFFSET_MAX_V         1.90f

/* 1 = d/q 电流闭环（目标单位 A），0 = 电压模式（目标单位 V） */
#define USE_CURRENT_LOOP      1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float motor_target_voltage = 1.0f;   /* 单位 V，用于电压模式及蜂鸣门控 */
volatile float motor_target_current = 0.06f;   /* 单位 A，直接作为 q 轴目标电流 */

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


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

static void SimpleFOC_Init(void)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;

  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
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
    Error_Handler();
  }

  if (CurrentSense_CalibrateOffset() != HAL_OK)
  {
    Error_Handler();
  }

  /* 通过已验证的硬件 I2C1 驱动读取 AS5600 */
  MagneticSensor_Init();

  /* 本板 FOC 参数：24 V 母线、4310 电机 */
  voltage_power_supply = 24.0f;
  voltage_limit = 4.0f;
  voltage_sensor_align = 4.0f;

#if USE_CURRENT_LOOP
  torque_controller = Type_foc_current;
#else
  torque_controller = Type_voltage;
#endif

  Motor_init();
  Motor_initFOC();

#if USE_CURRENT_LOOP
  /* 传感器校准会直接驱动电机各相，因此校准结束后才使能电流环。 */
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(800);//上电等待800ms，等电压稳定

  SimpleFOC_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    loopFOC();

    CurrentControl_SetTarget(motor_target_current);
    setTargetVotage(motor_target_voltage);

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
 * 以 PWM 频率（20 kHz）运行，每次 TIM2 TRGO 触发后执行一次。
 * 触发点位于计数器峰值前，此时处于三相低侧 MOS 管
 * 同时导通的窗口，电流流经 PGND 采样电阻。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1)
  {
    return;
  }

  adc_raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
  adc_raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);

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

  /*
   * STM32F1 HAL 的特殊处理（stm32f1xx_hal_adc.c 中的 HAL_ADC_IRQHandler）：
   * 常规组配置为软件启动、单次转换时，
   * （本项目未启用常规组，配置正是如此），HAL 会在
   * 首次注入组转换后关闭 JEOC 中断。
   * 硬件仍会在每次 TIM2 TRGO 时进行转换，因此只需重新使能中断。
   * 使能前先清除 JEOC 标志，以免误触发再次进入中断。
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
  /* 可在此处添加代码，报告 HAL 返回的错误状态 */
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
  /* 可在调试器中查看 file 和 line，定位断言失败的位置。 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
