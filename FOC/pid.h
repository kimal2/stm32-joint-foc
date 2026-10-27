/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Reusable PID controller (C port of SimpleFOC 2.4 PIDController).
  *
  * Features:
  *   - Tustin (trapezoidal) integral
  *   - integral and output limiting (anti-windup)
  *   - optional output slew-rate limit
  *   - caller provides dt, so it works with both fixed and variable loops
  *
  * Example:
  *     static PID_t pid;
  *     PID_Init(&pid, P, I, D, ramp, limit);
  *     u = PID_Update(&pid, reference - feedback, dt);
  ******************************************************************************
  */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float P;
  float I;
  float D;
  float output_ramp;    /* max output change rate [units/s], 0 = disabled   */
  float limit;          /* symmetric limit for integral and output, <=0 = off */
  float integral_prev;  /* internal state                                   */
  float error_prev;     /* internal state                                   */
  float output_prev;    /* internal state                                   */
} PID_t;

void  PID_Init(PID_t *pid, float P, float I, float D, float ramp, float limit);
void  PID_Reset(PID_t *pid);
void  PID_SetGains(PID_t *pid, float P, float I, float D);
void  PID_SetLimit(PID_t *pid, float limit);

/* Returns the controller output for this sample. dt in seconds. */
float PID_Update(PID_t *pid, float error, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
