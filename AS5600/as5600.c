/**
  ******************************************************************************
  * @file    as5600.c
  * @brief   AS5600 磁旋转编码器驱动（硬件 I2C1）
  ******************************************************************************
  * 读取 12 位原始角度寄存器（0x0C/0x0D，0..4095 对应 0..360 度），
  * 并将结果保存在全局变量中，方便调试器实时观察。
  ******************************************************************************
  */

#include "as5600.h"

#define AS5600_I2C_ADDR       ((uint16_t)(AS5600_ADDR7 << 1))   /* 0x6C */
#define AS5600_CPR            4096U
#define AS5600_TWO_PI         6.28318530718f
#define AS5600_I2C_TIMEOUT    5U    /* 毫秒 */
#define AS5600_READ_RETRIES   3U

/* 全局变量，可在调试器中观察 */
volatile uint16_t as5600_raw_angle = 0;
volatile float    as5600_angle_rad = 0.0f;
volatile float    as5600_angle_deg = 0.0f;
volatile uint8_t  as5600_status = 0;
volatile uint8_t  as5600_error = 1;

static I2C_HandleTypeDef *s_hi2c = NULL;

/* 读取一对大端寄存器组成的 16 位数据，失败时返回 0xFFFF。 */
static uint16_t AS5600_ReadTwoBytes(uint8_t reg)
{
  uint8_t buf[2];
  HAL_StatusTypeDef st;

  if (s_hi2c == NULL)
  {
    return 0xFFFFU;
  }

  st = HAL_I2C_Mem_Read(s_hi2c, AS5600_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                        buf, sizeof(buf), AS5600_I2C_TIMEOUT);
  if (st != HAL_OK)
  {
    return 0xFFFFU;
  }

  return (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

uint16_t AS5600_ReadRawAngle(void)
{
  uint16_t value = AS5600_ReadTwoBytes(AS5600_REG_RAW_ANGLE_H);
  return (value == 0xFFFFU) ? 0xFFFFU : (value & 0x0FFFU);
}

uint16_t AS5600_ReadAngle(void)
{
  uint16_t value = AS5600_ReadTwoBytes(AS5600_REG_ANGLE_H);
  return (value == 0xFFFFU) ? 0xFFFFU : (value & 0x0FFFU);
}

uint8_t AS5600_ReadStatus(void)
{
  uint8_t status = 0;

  if ((s_hi2c == NULL) ||
      (HAL_I2C_Mem_Read(s_hi2c, AS5600_I2C_ADDR, AS5600_REG_STATUS,
                        I2C_MEMADD_SIZE_8BIT, &status, 1, AS5600_I2C_TIMEOUT) != HAL_OK))
  {
    return 0xFFU;
  }

  return status;
}

uint8_t AS5600_Init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;

  as5600_error = 1;

  if (HAL_I2C_IsDeviceReady(hi2c, AS5600_I2C_ADDR, AS5600_READ_RETRIES,
                            AS5600_I2C_TIMEOUT) != HAL_OK)
  {
    return 1;
  }

  as5600_status = AS5600_ReadStatus();

  if (as5600_status == 0xFFU)
  {
    return 1;
  }

  as5600_error = 0;
  return AS5600_Update();
}

uint8_t AS5600_Update(void)
{
  uint16_t raw = AS5600_ReadRawAngle();

  if (raw == 0xFFFFU)
  {
    as5600_error = 1;
    return 1;
  }

  as5600_raw_angle = raw;
  as5600_angle_rad = ((float)raw * AS5600_TWO_PI) / (float)AS5600_CPR;
  as5600_angle_deg = ((float)raw * 360.0f) / (float)AS5600_CPR;
  as5600_error = 0;

  return 0;
}
