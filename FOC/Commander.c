/**
  ******************************************************************************
  * @file    Commander.c
  * @brief   Tiny text command parser on USART1 RX (non-blocking, polled).
  ******************************************************************************
  */

#include "Commander.h"
#include "usart.h"

/* Tiny float parser: [+-]digits[.digits], far cheaper than strtof() */
static float ParseFloat(const char *s)
{
  float sign = 1.0f;
  float value = 0.0f;
  float frac = 0.0f;
  float div = 1.0f;

  if (*s == '-')
  {
    sign = -1.0f;
    s++;
  }
  else if (*s == '+')
  {
    s++;
  }

  while ((*s >= '0') && (*s <= '9'))
  {
    value = value * 10.0f + (float)(*s - '0');
    s++;
  }

  if (*s == '.')
  {
    s++;
    while ((*s >= '0') && (*s <= '9'))
    {
      frac = frac * 10.0f + (float)(*s - '0');
      div *= 10.0f;
      s++;
    }
  }

  return sign * (value + frac / div);
}

/* Targets live in main.c */
extern volatile float motor_target_speed;
extern volatile float motor_target_current;
extern volatile float motor_target_voltage;
extern volatile float motor_target_position;

#define CMD_BUF_LEN 32U

static char    cmd_buf[CMD_BUF_LEN];
static uint8_t cmd_len = 0U;

void Commander_Init(void)
{
  cmd_len = 0U;
}

void Commander_Poll(void)
{
  uint8_t ch;

  /* Non-blocking RX: reading SR then DR clears RXNE/ORE */
  if ((huart1.Instance->SR & USART_SR_RXNE) == 0U)
  {
    return;
  }
  ch = (uint8_t)(huart1.Instance->DR & 0xFFU);

  if ((ch == '\r') || (ch == '\n'))
  {
    if (cmd_len > 0U)
    {
      cmd_buf[cmd_len] = '\0';

      switch (cmd_buf[0])
      {
        case 's':
        case 'S':
          motor_target_speed = ParseFloat(&cmd_buf[1]);
          break;
        case 'i':
        case 'I':
          motor_target_current = ParseFloat(&cmd_buf[1]);
          break;
        case 'v':
        case 'V':
          motor_target_voltage = ParseFloat(&cmd_buf[1]);
          break;
        case 'p':
        case 'P':
          motor_target_position = ParseFloat(&cmd_buf[1]);
          break;
        default:
          break;
      }
      cmd_len = 0U;
    }
  }
  else if (cmd_len < (CMD_BUF_LEN - 1U))
  {
    cmd_buf[cmd_len++] = (char)ch;
  }
}
