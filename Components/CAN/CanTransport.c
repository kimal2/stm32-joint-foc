#include "CanTransport.h"

/* 电流命令只需要最新值：新帧覆盖尚未处理的旧帧，不建立接收队列。 */
static CanFrame rx_frame;
static volatile uint8_t rx_ready;

volatile uint32_t can_rx_count;
volatile uint32_t can_tx_count;
volatile uint32_t can_tx_busy_count;
volatile uint32_t can_hal_error;

HAL_StatusTypeDef CanTransport_Init(uint16_t receive_id)
{
  CAN_FilterTypeDef filter = {0};
  /* 只接受指定标准数据帧，屏蔽扩展帧和远程帧。 */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (uint32_t)receive_id << 5;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0xFFE0U;
  filter.FilterMaskIdLow = 0x0006U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  rx_ready = 0U;
  can_rx_count = can_tx_count = can_tx_busy_count = 0U;
  can_hal_error = 0U;
  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) return HAL_ERROR;
  if (HAL_CAN_Start(&hcan) != HAL_OK) return HAL_ERROR;
  /* NVIC 由 CubeMX 开启，这里还要开启 CAN 外设内部的 FIFO0 接收通知。 */
  return HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_rx)
{
  CAN_RxHeaderTypeDef header;
  CanFrame frame;
  if (hcan_rx != &hcan) return;

  /* 读出 FIFO 内所有待收帧，清空后接收中断自然结束；不在这里解析或控制电机。 */
  while (HAL_CAN_GetRxFifoFillLevel(hcan_rx, CAN_RX_FIFO0) != 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan_rx, CAN_RX_FIFO0, &header, frame.data) != HAL_OK)
    {
      can_hal_error = HAL_CAN_GetError(hcan_rx);
      return;
    }
    frame.extended = (header.IDE == CAN_ID_EXT);
    frame.remote = (header.RTR == CAN_RTR_REMOTE);
    frame.id = frame.extended ? header.ExtId : header.StdId;
    frame.length = (uint8_t)header.DLC;
    frame.received_tick = HAL_GetTick();
    rx_frame = frame;
    __DMB();  /* 完整写入报文后，再发布“有新帧”标志。 */
    rx_ready = 1U;
    can_rx_count++;
  }
}

HAL_StatusTypeDef CanTransport_Receive(CanFrame *frame)
{
  if (rx_ready == 0U) return HAL_BUSY;

  /* 复制期间短暂屏蔽中断，防止得到前后两帧拼接的数据；随后恢复原中断状态。 */
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  *frame = rx_frame;
  rx_ready = 0U;
  __set_PRIMASK(irq_state);
  return HAL_OK;
}

HAL_StatusTypeDef CanTransport_Send(uint16_t id, const uint8_t data[8])
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
  {
    can_tx_busy_count++;
    return HAL_BUSY;
  }
  header.StdId = id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
  if (result == HAL_OK) can_tx_count++;
  else can_hal_error = HAL_CAN_GetError(&hcan);
  return result;
}
