/**
  ******************************************************************************
  * @file    PositionControl.h
  * @brief   Outer position (angle) loop.
  *
  * Cascade:
  *     position PID -> speed reference [rad/s]
  *                  -> speed PI -> iq reference
  *                  -> current loop -> SVPWM
  *
  * Runs from the main loop at ~1 kHz, same tick as the speed loop.
  ******************************************************************************
  */

#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void  PositionControl_Init(void);
void  PositionControl_Reset(void);

/* Target mechanical shaft angle in rad (multi-turn, same frame as shaft_angle) */
void  PositionControl_SetTarget(float rad);

/* measured_rad: shaft_angle, dt: seconds since the last call.
   Returns the speed reference in rad/s. */
float PositionControl_Update(float measured_rad, float dt);

#ifdef __cplusplus
}
#endif

#endif /* POSITIONCONTROL_H */
