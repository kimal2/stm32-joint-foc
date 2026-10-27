/**
  ******************************************************************************
  * @file    beep.h
  * @brief   "Beeper" using the motor windings, ported from the sample project
  ******************************************************************************
  * TIM4 runs at 1 MHz. Its update interrupt flips the stator voltage vector
  * between two positions, producing an audible tone in the motor.
  ******************************************************************************
  */

#ifndef __BEEP_H
#define __BEEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Note period in microseconds (same table as the sample project) */
enum
{
  T_None = 0,
  /* low octave */
  T_L1 = 3822, T_L2 = 3405, T_L3 = 3034, T_L4 = 2863,
  T_L5 = 2551, T_L6 = 2272, T_L7 = 2052,
  /* middle octave */
  T_M1 = 1911, T_M2 = 1703, T_M3 = 1517, T_M4 = 1432,
  T_M5 = 1276, T_M6 = 1136, T_M7 = 1012,
  /* high octave */
  T_H1 = 956,  T_H2 = 851,  T_H3 = 758,  T_H4 = 716,
  T_H5 = 638,  T_H6 = 568,  T_H7 = 506
};

/* Configure TIM4 (1 MHz time base) and its interrupt. */
void Beep_Init(void);

/* period_us == 0 stops the tone. */
void Beep_Play(uint16_t period_us);

/* Play num notes, each {period_us, duration_ms}. Blocking. */
void Beep_PlayNotes(uint8_t num, const uint16_t notes[][2]);

#ifdef __cplusplus
}
#endif

#endif /* __BEEP_H */
