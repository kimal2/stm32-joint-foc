/**
  ******************************************************************************
  * @file    SpeedControl.c
  * @brief   Outer speed (velocity) PI loop.
  ******************************************************************************
  */

#include "SpeedControl.h"
#include "foc_utils.h"
#include "pid.h"

/*
 * GB4310: J = 202 g*cm^2 = 2.02e-5 kg*m^2, Kt = 0.23 N*m/A
 *
 * PI design for the mechanical plant Kt/(J*s):
 *     Kp = 2*zeta*wn*J/Kt
 *     Ki = wn^2*J/Kt
 *
 *   5 Hz,  zeta = 0.707 : Kp = 0.0039 A/(rad/s), Ki = 0.087 A*s/rad
 *  10 Hz,  zeta = 0.707 : Kp = 0.0078 A/(rad/s), Ki = 0.347 A*s/rad
 *
 * Start at 5 Hz. If the load inertia is much larger than the rotor,
 * Kp/Ki will need to be larger (tune Kp first, then Ki).
 */
#define SPEED_KP      0.004f
#define SPEED_KI      0.087f
#define SPEED_LIMIT   1.0f      /* A, keep <= the current loop limit */

static float target_rad_s = 0.0f;
static PID_t pid;

void SpeedControl_Init(void)
{
  target_rad_s = 0.0f;
  PID_Init(&pid, SPEED_KP, SPEED_KI, 0.0f, 0.0f, SPEED_LIMIT);
}

void SpeedControl_Reset(void)
{
  PID_Reset(&pid);
}

void SpeedControl_SetTarget(float rad_s)
{
  target_rad_s = _constrain(rad_s, -10000.0f, 10000.0f);
}

float SpeedControl_Update(float measured_rad_s, float dt)
{
  /* guard against a bad dt (first call, blocked main loop, ...) */
  if (dt < 0.0005f)
  {
    dt = 0.0005f;
  }
  else if (dt > 0.02f)
  {
    dt = 0.02f;
  }

  return PID_Update(&pid, target_rad_s - measured_rad_s, dt);
}
