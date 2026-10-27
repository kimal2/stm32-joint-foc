/**
  ******************************************************************************
  * @file    retarget.c
  * @brief   printf() output on USART1 (PA9 TX), 115200 8N1
  ******************************************************************************
  * Core/Src/syscalls.c declares __io_putchar() weak, so this definition makes
  * the printf() calls in the ported FOC code visible on the serial port.
  ******************************************************************************
  */

#include "usart.h"

int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;

  HAL_UART_Transmit(&huart1, &c, 1U, 10U);

  return ch;
}
