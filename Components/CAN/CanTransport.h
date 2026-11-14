#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H

#include "can.h"

typedef struct
{
  uint32_t id;
  uint8_t extended;
  uint8_t remote;
  uint8_t length;
  uint8_t data[8];
  uint32_t received_tick;  /* 中断收到报文时的毫秒计数，用于命令超时 */
} CanFrame;

/* 组件仅提供过滤、启动和非阻塞收发，不解释电机协议。
 * FIFO0 中断缓存最新报文，主循环取出完整快照；发送不在软件中排队。
 */
HAL_StatusTypeDef CanTransport_Init(uint16_t receive_id);
HAL_StatusTypeDef CanTransport_Receive(CanFrame *frame);
HAL_StatusTypeDef CanTransport_Send(uint16_t id, const uint8_t data[8]);
extern volatile uint32_t can_rx_count;
extern volatile uint32_t can_tx_count;
extern volatile uint32_t can_tx_busy_count;
extern volatile uint32_t can_hal_error;

#endif
