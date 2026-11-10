/**
  ******************************************************************************
  * @file    as5600.h
  * @brief   AS5600 磁旋转编码器驱动（硬件 I2C1）
  ******************************************************************************
  * 硬件连接（依据原理图网络表）：
  *   AS5600 SCL (U6.7) -> PB6  (I2C1_SCL)
  *   AS5600 SDA (U6.6) -> PB7  (I2C1_SDA)
  *   3.3 V 供电，两条总线均使用 4.7 千欧上拉电阻。
  ******************************************************************************
  */

#ifndef __AS5600_H
#define __AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"
#include <stdint.h>

/* AS5600 的 7 位 I2C 地址 */
#define AS5600_ADDR7              0x36U

/* 本驱动使用的寄存器地址 */
#define AS5600_REG_STATUS         0x0BU
#define AS5600_REG_RAW_ANGLE_H    0x0CU
#define AS5600_REG_ANGLE_H        0x0EU

/* STATUS 寄存器位定义 */
#define AS5600_STATUS_MH          (1U << 3)   /* 磁场过强 */
#define AS5600_STATUS_ML          (1U << 4)   /* 磁场过弱   */
#define AS5600_STATUS_MD          (1U << 5)   /* 已检测到磁铁   */

/*
 * 用于 IDE 实时调试的全局变量。
 *   as5600_raw_angle   ：原始角度 0..4095，由 AS5600_Update() 更新
 *   as5600_angle_rad   ：换算后的弧度值，范围 0..2*pi
 *   as5600_angle_deg   ：换算后的角度值，范围 0..360
 *   as5600_status      ：STATUS 寄存器（0x0B）原始值
 *   as5600_error       ：最近一次 I2C 通信结果，0 = 成功，1 = 失败
 */
extern volatile uint16_t as5600_raw_angle;
extern volatile float    as5600_angle_rad;
extern volatile float    as5600_angle_deg;
extern volatile uint8_t  as5600_status;
extern volatile uint8_t  as5600_error;

/* 初始化驱动，成功返回 0，传感器无响应时返回 1。 */
uint8_t AS5600_Init(I2C_HandleTypeDef *hi2c);

/* 读取一次新数据并更新上述全局变量，成功返回 0。 */
uint8_t AS5600_Update(void);

/* 原始寄存器访问接口；16 位角度读取失败时返回 0xFFFF。 */
uint16_t AS5600_ReadRawAngle(void);
uint16_t AS5600_ReadAngle(void);
uint8_t  AS5600_ReadStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H */
