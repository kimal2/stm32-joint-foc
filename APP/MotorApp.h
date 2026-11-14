#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include "main.h"

typedef enum
{
  MOTOR_APP_INITIALIZING = 0,
  MOTOR_APP_RUNNING,
  MOTOR_APP_DISABLED,
  MOTOR_APP_TIMEOUT,
  MOTOR_APP_FAULT
} MotorAppState;

extern volatile MotorAppState motor_app_state;
extern volatile float motor_target_current;
extern volatile float motor_target_voltage;
extern volatile float motor_velocity_rad_s;
extern volatile uint32_t motor_command_elapsed_ms;
extern volatile uint32_t motor_command_count;

/* 保留原调试变量名称，便于 Ozone 观察。 */
extern volatile uint16_t adc_raw_a, adc_raw_b;
extern volatile float phase_current_a, phase_current_b, phase_current_c;
extern volatile uint32_t adc_sample_count;
extern volatile float current_offset_adc_a, current_offset_adc_b;
extern volatile uint8_t current_offset_status;
extern volatile uint32_t current_offset_sample_count;

/* 外设初始化完成后调用，执行零点与电角度校准，默认以 0 A 使能。 */
HAL_StatusTypeDef MotorApp_Init(void);
/* 主循环调用：读取编码器、更新上报速度。 */
void MotorApp_Update(void);
/* SysTick 每 1 ms 调用：增加命令计数，超时时关闭使能引脚并清零目标电流。 */
void MotorApp_OnSysTick1ms(void);
/* 仅由 main.c 的 ADC 注入组回调调用，保留原二分频电流环。 */
void MotorApp_OnAdcSample(uint16_t raw_a, uint16_t raw_b);
/* 合法电流帧解析后调用；received_tick 是中断接收时间，不是解析时间。
 * 运行时应用目标并清零计数；命令超时状态下收到新指令时自动恢复使能。
 * 当前调用方传入的电流来自 int16_t 毫安数值。
 */
HAL_StatusTypeDef MotorApp_ApplyCurrent(float current_a, uint32_t received_tick);
void MotorApp_Disable(MotorAppState reason);
/* 本地零目标使能接口；命令超时也可通过新的合法电流帧自动恢复。 */
HAL_StatusTypeDef MotorApp_Enable(void);

#endif
