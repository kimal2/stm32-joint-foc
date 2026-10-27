/**
  ******************************************************************************
  * @file    Vofa.c
  * @brief   VOFA+ "JustFloat" frame sender for USART1.
  ******************************************************************************
  */

#include "Vofa.h"
#include "usart.h"
#include <string.h>

/* +inf, little-endian: 00 00 80 7F */
static const uint8_t vofa_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

void VOFA_Send(const float *data, uint8_t ch_count)
{
  uint8_t buf[VOFA_MAX_CH * 4U + 4U];
  size_t  n;
  uint16_t len;

  if ((data == NULL) || (ch_count == 0U))
  {
    return;
  }

  if (ch_count > VOFA_MAX_CH)
  {
    ch_count = (uint8_t)VOFA_MAX_CH;
  }

  /* Cortex-M3 is little-endian, so the float bytes are already in the
     byte order VOFA+ expects. */
  n = (size_t)ch_count * sizeof(float);
  memcpy(buf, data, n);
  memcpy(&buf[n], vofa_tail, sizeof(vofa_tail));
  len = (uint16_t)(n + sizeof(vofa_tail));

  HAL_UART_Transmit(&huart1, buf, len, 10U);
}
