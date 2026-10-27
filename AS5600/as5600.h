/**
  ******************************************************************************
  * @file    as5600.h
  * @brief   AS5600 magnetic rotary encoder driver (hardware I2C1)
  ******************************************************************************
  * Wiring (from schematic netlist):
  *   AS5600 SCL (U6.7) -> PB6  (I2C1_SCL)
  *   AS5600 SDA (U6.6) -> PB7  (I2C1_SDA)
  *   3.3V supply, 4.7K pull-ups on both lines.
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

/* 7-bit I2C address of AS5600 */
#define AS5600_ADDR7              0x36U

/* Register map (relevant subset) */
#define AS5600_REG_STATUS         0x0BU
#define AS5600_REG_RAW_ANGLE_H    0x0CU
#define AS5600_REG_ANGLE_H        0x0EU

/* STATUS register bits */
#define AS5600_STATUS_MH          (1U << 3)   /* magnet too strong */
#define AS5600_STATUS_ML          (1U << 4)   /* magnet too weak   */
#define AS5600_STATUS_MD          (1U << 5)   /* magnet detected   */

/*
 * Globals for live debugging in the IDE.
 *   as5600_raw_angle   : 0..4095, updated by AS5600_Update()
 *   as5600_angle_rad   : 0..2*pi, same value converted
 *   as5600_angle_deg   : 0..360, same value converted
 *   as5600_status      : raw STATUS register (0x0B)
 *   as5600_error       : 0 = last I2C transaction OK, 1 = failed
 */
extern volatile uint16_t as5600_raw_angle;
extern volatile float    as5600_angle_rad;
extern volatile float    as5600_angle_deg;
extern volatile uint8_t  as5600_status;
extern volatile uint8_t  as5600_error;

/* Initialize driver; returns 0 on success, 1 if the sensor does not answer. */
uint8_t AS5600_Init(I2C_HandleTypeDef *hi2c);

/* Read one fresh sample into the globals above; returns 0 on success. */
uint8_t AS5600_Update(void);

/* Raw register access helpers. Return 0xFFFF on error. */
uint16_t AS5600_ReadRawAngle(void);
uint16_t AS5600_ReadAngle(void);
uint8_t  AS5600_ReadStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H */
