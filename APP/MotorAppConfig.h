#ifndef MOTOR_APP_CONFIG_H
#define MOTOR_APP_CONFIG_H

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


#define MOTOR_APP_DEFAULT_ID 1U
_Static_assert(MOTOR_APP_DEFAULT_ID >= 1U && MOTOR_APP_DEFAULT_ID <= 4U,
               "MOTOR_APP_DEFAULT_ID must be 1..4");
/* 单位 ms：500 ms 无有效命令则失能；收到新的合法电流指令后自动恢复。 */
#define MOTOR_COMMAND_TIMEOUT_MS 500U
/* CAN 反馈由每 1 ms 的 SysTick 直接提交，频率为 1 kHz。 */

#endif
