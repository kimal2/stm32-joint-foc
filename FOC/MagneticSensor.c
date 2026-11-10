/************************************************
 * 为移植后的 SimpleFOC 封装 AS5600 磁编码器接口。
 *
 * 原示例在 PB6/PB7 上使用软件 I2C；本板已配置并验证
 * 硬件 I2C1（PB6/PB7），因此保持公共接口一致，
 * 底层改用 AS5600/as5600.c 中的硬件驱动。
 ************************************************/

#include "MagneticSensor.h"
#include "foc_utils.h"
#include "as5600.h"
#include "i2c.h"
#include <math.h>

/******************************************************************************/
long cpr;
float full_rotation_offset;
long angle_data, angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/******************************************************************************/
#define AS5600_CPR 4096

/* 速度估算：根据 HAL 毫秒时基计算角度差分，再进行
   一阶低通滤波，Tf = 10 ms，对应约 16 Hz 截止频率。 */
#define VELOCITY_LPF_TF 0.01f

static float   angle_cont = 0.0f;
static float   velocity_filtered = 0.0f;
static float   angle_vel_prev = 0.0f;
static uint32_t vel_tick_prev = 0U;
static uint8_t  vel_init = 0U;

/* 传感器方向由 BLDCMotor.c 中的 alignSensor() 校准。
   速度乘以该方向系数后，与机械轴（FOC）的正方向一致。 */
extern long sensor_direction;

void MagneticSensor_Init(void)
{
  cpr = AS5600_CPR;

  AS5600_Init(&hi2c1);

  angle_data = (long)AS5600_ReadRawAngle();
  if (angle_data == 0xFFFFL)
  {
    angle_data = 0;
  }
  angle_data_prev = angle_data;
  full_rotation_offset = 0.0f;
  velocity_calc_timestamp = 0;
  angle_prev = 0.0f;

  angle_cont = 0.0f;
  velocity_filtered = 0.0f;
  angle_vel_prev = 0.0f;
  vel_init = 0U;
}

/*
 * 返回连续的机械角度，单位 rad；通过跟踪传感器的整圈转数，
 * 使角度范围不再局限于 0 到 2*pi。
 */
float getAngle(void)
{
  uint16_t raw;
  long d_angle;

  /* 调用共用驱动更新函数，同时更新 as5600_* 调试变量 */
  if (AS5600_Update() != 0)
  {
    /* I2C 读取失败时沿用上一次角度，避免引入错误角度 */
    raw = (uint16_t)angle_data_prev;
  }
  else
  {
    raw = as5600_raw_angle;
  }

  angle_data = (long)(raw & 0x0FFFU);
  d_angle = angle_data - angle_data_prev;

  /* 检测 0 与 4095 之间的跨圈跳变，并补偿一整圈角度 */
  if (fabsf((float)d_angle) > 0.8f * (float)cpr)
  {
    full_rotation_offset += (d_angle > 0) ? -_2PI : _2PI;
  }

  angle_data_prev = angle_data;

  angle_cont = full_rotation_offset +
               ((float)angle_data * _2PI / (float)cpr);
  return angle_cont;
}

/*
 * 机械轴速度，单位 rad/s，正方向与 FOC 机械轴方向一致。
 * 需要估算速度时，应先更新角度，再按较稳定的周期调用。
 */
float getVelocity(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t dt_ms;
  float dt;
  float v;
  float alpha;

  if (vel_init == 0U)
  {
    angle_vel_prev = angle_cont;
    vel_tick_prev = now;
    vel_init = 1U;
    return 0.0f;
  }

  dt_ms = now - vel_tick_prev;
  if (dt_ms == 0U)
  {
    return velocity_filtered;
  }

  dt = (float)dt_ms * 0.001f;
  v = ((angle_cont - angle_vel_prev) * (float)sensor_direction) / dt;

  alpha = dt / (VELOCITY_LPF_TF + dt);
  velocity_filtered += alpha * (v - velocity_filtered);

  angle_vel_prev = angle_cont;
  vel_tick_prev = now;

  return velocity_filtered;
}
