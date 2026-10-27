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

  return full_rotation_offset +
         ((float)angle_data * _2PI / (float)cpr);
}

/* Not used by the ported FOC core; kept for API compatibility. */
float getVelocity(void)
{
  return 0.0f;
}
