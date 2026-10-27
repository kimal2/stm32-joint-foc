/**
  ******************************************************************************
  * @file    Commander.h
  * @brief   Tiny text command parser on USART1 RX (non-blocking, polled).
  *
  * Commands (terminated by \n or \r), sent from VOFA+ or any serial terminal:
  *     s<float>   set motor_target_speed      [rad/s]
  *     i<float>   set motor_target_current    [A]
  *     v<float>   set motor_target_voltage    [V]
  *     p<float>   set motor_target_position   [rad, multi-turn]
  * Examples: "s10", "s-5.5", "i0.2", "v1.0", "p3.14"
  *
  * Nothing is echoed back: text on USART1 TX would corrupt the VOFA frames.
  ******************************************************************************
  */

#ifndef COMMANDER_H
#define COMMANDER_H

#ifdef __cplusplus
extern "C" {
#endif

void Commander_Init(void);
void Commander_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDER_H */
