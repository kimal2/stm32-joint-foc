/************************************************
 * AS5600 magnetic sensor wrapper for the ported SimpleFOC code.
 *
 * The original sample used bit-banged I2C on PB6/PB7. This board already
 * has I2C1 (PB6/PB7) configured and verified, so the same public API is
 * implemented on top of the hardware driver in Devices/as5600.c.
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

/* Velocity estimation: angle difference over the HAL tick (1 ms), then a
   first order low-pass filter. Tf = 10 ms -> ~16 Hz cutoff. */
#define VELOCITY_LPF_TF 0.01f

static float   angle_cont = 0.0f;
static float   velocity_filtered = 0.0f;
static float   angle_vel_prev = 0.0f;
static uint32_t vel_tick_prev = 0U;
static uint8_t  vel_init = 0U;

/* Sensor direction is calibrated by alignSensor() in BLDCMotor.c.
   Multiplying by it puts the velocity in the shaft (FOC) frame. */
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
 * Returns a continuous mechanical angle in radians: it tracks how many full
 * rotations the sensor has made, so the range is not limited to 0..2*pi.
 */
float getAngle(void)
{
  uint16_t raw;
  long d_angle;

  /* Use the shared driver update so as5600_* debug globals stay fresh too */
  if (AS5600_Update() != 0)
  {
    /* I2C error: keep the last value instead of injecting a bogus angle */
    raw = (uint16_t)angle_data_prev;
  }
  else
  {
    raw = as5600_raw_angle;
  }

  angle_data = (long)(raw & 0x0FFFU);
  d_angle = angle_data - angle_data_prev;

  /* Detect the 0 <-> 4095 wrap and account for one full rotation */
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
 * Mechanical shaft velocity in rad/s (in the FOC/shaft direction).
 * Call it at a fixed-ish rate (the main loop does it once per millisecond).
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
