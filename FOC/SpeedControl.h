/**
  ******************************************************************************
  * @file    SpeedControl.h
  * @brief   Outer speed (velocity) PI loop.
  *
  * Output is the iq reference for the d/q current loop:
  *     speed PI -> iq_ref -> CurrentControl -> setPhaseVoltage
  * Runs from the main loop at ~1 kHz (HAL tick).
  ******************************************************************************
  */

#ifndef SPEEDCONTROL_H
#define SPEEDCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void  SpeedControl_Init(void);
void  SpeedControl_Reset(void);

/* Target mechanical speed in rad/s */
void  SpeedControl_SetTarget(float rad_s);

/* measured_rad_s: shaft velocity, dt: seconds since the last call.
   Returns the iq reference in A. */
float SpeedControl_Update(float measured_rad_s, float dt);

#ifdef __cplusplus
}
#endif

#endif /* SPEEDCONTROL_H */
