/**
  ******************************************************************************
  * @file    CurrentControl.h
  * @brief   d/q 电流闭环控制（基于 SimpleFOC 2.4 结构的 C 语言移植）
  *
  * 在 ADC1 注入组中断中以约 10 kHz 执行（每 2 次 20 kHz 采样执行一次）。
  * 根据 GB4310 数据手册，使用 SimpleFOC 的
  * tuneCurrentController() 公式计算控制器增益：
  *     比例增益 P = L * 2*PI * 带宽
  *     积分增益 I = R * 2*PI * 带宽
  *     低通滤波时间常数 Tf = 1 / (2*PI * 带宽 * 5)
  ******************************************************************************
  */

#ifndef CURRENTCONTROL_H
#define CURRENTCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 滤波后的 d/q 轴实测电流，用于调试和数据上传 */
extern volatile float current_d;
extern volatile float current_q;

/* 最近一次有效电流环计算的原始 CPU 周期数，不换算时间单位。
 * 不含 ADC 采样换算、中断进出及电流环入口检查；未执行电流环时保留上次值。
 * IQmath 版包含浮点接口与 Q20 的格式转换；由 CMake 的 FOC_USE_IQMATH 选择实现。
 */
extern volatile uint32_t current_loop_cycles;

void CurrentControl_Init(void);
void CurrentControl_Enable(uint8_t enable);

/* q 轴目标电流，单位 A（正值对应正向转矩），限幅为 ±CURRENT_LIMIT */
void CurrentControl_SetTarget(float iq_target);

/*
 * 执行一次电流环迭代。
 *   ia, ib：相电流，单位 A，以流入电机为正方向
 *   theta_e：电角度，单位 rad
 * 仅在 ADC 注入组转换完成回调中调用。
 */
void CurrentControl_Update(float ia, float ib, float theta_e);

#ifdef __cplusplus
}
#endif

#endif /* CURRENTCONTROL_H */
