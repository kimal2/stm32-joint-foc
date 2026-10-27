/**
  ******************************************************************************
  * @file    Vofa.h
  * @brief   VOFA+ "JustFloat" frame sender for USART1.
  *
  * Frame format (little-endian):
  *   float ch0 | float ch1 | ... | float chN-1 | 00 00 80 7F
  *
  * The 4 byte tail (0x7F800000 = +inf) is the frame delimiter that VOFA+
  * uses in its JustFloat parser.
  ******************************************************************************
  */

#ifndef __VOFA_H
#define __VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Upper bound for the channel count (used for the internal TX buffer) */
#define VOFA_MAX_CH 8U

/*
 * Send one frame on USART1.
 *   data     : pointer to ch_count floats
 *   ch_count : 1 .. VOFA_MAX_CH
 *
 * Blocking call (HAL_UART_Transmit), so data can be a local array.
 * Do not call it from an interrupt.
 */
void VOFA_Send(const float *data, uint8_t ch_count);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H */
