/**
  ******************************************************************************
  * @file    CurrentControl.h
  * @brief   Simple d/q current closed loop (SimpleFOC 2.4 structure, C port).
  *
  * Runs from the ADC1 injected interrupt at 5 kHz (every 4th 20 kHz sample).
  * Gains are derived from the GB4310 datasheet with SimpleFOC's
  * tuneCurrentController() formula:
  *     P = L * 2*PI * bandwidth
  *     I = R * 2*PI * bandwidth
  *     LPF Tf = 1 / (2*PI * bandwidth * 5)
  ******************************************************************************
  */

#ifndef CURRENTCONTROL_H
#define CURRENTCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Filtered measured d/q currents, for debugger / telemetry */
extern volatile float current_d;
extern volatile float current_q;

void CurrentControl_Init(void);
void CurrentControl_Enable(uint8_t enable);

/* iq reference in A (positive = positive torque), clamped to +-CURRENT_LIMIT */
void CurrentControl_SetTarget(float iq_target);

/*
 * One current-loop iteration.
 *   ia, ib : phase currents (A, positive into the motor)
 *   theta_e: electrical angle (rad)
 * Call it from the ADC injected callback only.
 */
void CurrentControl_Update(float ia, float ib, float theta_e);

#ifdef __cplusplus
}
#endif

#endif /* CURRENTCONTROL_H */
