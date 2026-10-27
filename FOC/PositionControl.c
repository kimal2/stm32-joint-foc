/**
  ******************************************************************************
  * @file    PositionControl.c
  * @brief   Outer position (angle) loop.
  ******************************************************************************
  */

#include "PositionControl.h"
#include "foc_utils.h"
#include "pid.h"

/*
 * The inner speed loop is closed, so the position plant is roughly 1/s.
 * A P controller gives a first-order response with bandwidth = Kp [rad/s]:
 *
 *   Kp = 10  -> ~1.6 Hz position bandwidth
 *   Kp = 20  -> ~3.2 Hz
 *
 * Ki = 0 is enough because the speed loop already has an integrator
 * (no steady-state position error for a step).
 *
 * Output (speed reference) is limited to 20 rad/s (~190 rpm), the same
 * default SimpleFOC uses for DEF_VEL_LIM.
 */
#define POS_KP           10.0f      /* 1/s        */
#define POS_KI           0.0f
#define POS_KD           0.0f
#define POS_SPEED_LIMIT  20.0f      /* rad/s      */

static PID_t pid;
static float target_rad = 0.0f;

void PositionControl_Init(void)
{
  target_rad = 0.0f;
  PID_Init(&pid, POS_KP, POS_KI, POS_KD, 0.0f, POS_SPEED_LIMIT);
}

void PositionControl_Reset(void)
{
  PID_Reset(&pid);
}

void PositionControl_SetTarget(float rad)
{
  target_rad = _constrain(rad, -100000.0f, 100000.0f);
}

float PositionControl_Update(float measured_rad, float dt)
{
  if (dt < 0.0005f)
  {
    dt = 0.0005f;
  }
  else if (dt > 0.02f)
  {
    dt = 0.02f;
  }

  return PID_Update(&pid, target_rad - measured_rad, dt);
}
