#include "CanApp.h"
#include "MotorApp.h"
#include "MotorAppConfig.h"
#include "MagneticSensor.h"
#include "FlashStorage.h"
#include <math.h>
#include <limits.h>

volatile uint8_t motor_id = MOTOR_APP_DEFAULT_ID;
volatile uint32_t can_command_rejected_count;
/* CAN 初始化成功后才允许 SysTick 发送，避免上电校准期间访问未启动的 CAN。 */
static volatile uint8_t feedback_enabled;

static int32_t EncodeSigned(float value, float scale, int32_t min_value, int32_t max_value)
{
  float scaled = value * scale;
  if (!isfinite(scaled)) return 0;
  /* INT32_MAX 转 float 会舍入到 2^31，必须先比较再转整数。 */
  if (scaled >= (float)max_value) return max_value;
  if (scaled <= (float)min_value) return min_value;
  return (int32_t)roundf(scaled);
}

static void Pack16(uint8_t *data, int32_t value)
{
  uint16_t bits = (uint16_t)value;
  data[0] = (uint8_t)(bits >> 8);
  data[1] = (uint8_t)bits;
}

void CanApp_PackMotorFeedback(uint8_t data[8], float angle_rad, float velocity_rad_s)
{
  /* 大端补码：连续机械角度占 4 字节（mrad），速度占 2 字节（0.01 rad/s）。 */
  uint32_t angle = (uint32_t)EncodeSigned(angle_rad, 1000.0f, INT32_MIN, INT32_MAX);
  data[0] = (uint8_t)(angle >> 24);
  data[1] = (uint8_t)(angle >> 16);
  data[2] = (uint8_t)(angle >> 8);
  data[3] = (uint8_t)angle;
  Pack16(&data[4], EncodeSigned(velocity_rad_s, 100.0f, INT16_MIN, INT16_MAX));
  data[6] = 0U;  /* 保留字段固定填零，当前不上报电流。 */
  data[7] = 0U;
}

HAL_StatusTypeDef CanApp_Init(void)
{
  feedback_enabled = 0U;
  /* ID 只在启动时确定，未保存有效值时使用板级默认值。 */
  uint8_t saved_id = Flash_ReadMotorID();
  motor_id = ((saved_id >= 1U) && (saved_id <= 4U)) ? saved_id : MOTOR_APP_DEFAULT_ID;
  can_command_rejected_count = 0U;
  HAL_StatusTypeDef result = CanTransport_Init(CAN_APP_CURRENT_ID);
  feedback_enabled = (result == HAL_OK);
  return result;
}

HAL_StatusTypeDef CanApp_ParseCurrentCommand(const CanFrame *frame)
{
  uint8_t id = motor_id;
  /* 只应用本协议的 8 字节标准数据帧，其他帧不刷新命令有效期。 */
  if ((frame->id != CAN_APP_CURRENT_ID) || (frame->extended != 0U) ||
      (frame->remote != 0U) || (frame->length != 8U))
  {
    can_command_rejected_count++;
    return HAL_ERROR;
  }
  uint8_t index = 2U * (id - 1U);
  /* 每两个字节对应一台电机：高字节在前，补码表示正负电流，1 LSB = 1 mA。 */
  uint16_t bits = ((uint16_t)frame->data[index] << 8) | frame->data[index + 1U];
  int32_t current_ma = (bits & 0x8000U) ? (int32_t)bits - 65536 : (int32_t)bits;
  HAL_StatusTypeDef result = MotorApp_ApplyCurrent((float)current_ma * 0.001f,
                                                frame->received_tick);
  if (result != HAL_OK) can_command_rejected_count++;
  return result;
}

void CanApp_ProcessReceivedCommands(void)
{
  CanFrame frame;
  /* FIFO 由中断读取；主循环只取一次最新缓存，避免执行积压的旧电流指令。 */
  if (CanTransport_Receive(&frame) == HAL_OK)
  {
    (void)CanApp_ParseCurrentCommand(&frame);
  }
}

void CanApp_SendMotorFeedback(void)
{
  if (feedback_enabled == 0U) return;

  uint8_t data[8];
  /* SysTick 只使用主循环更新的角度和速度缓存，不读取 I2C，不执行 loopFOC。 */
  CanApp_PackMotorFeedback(data, encoder_angle_report_rad, motor_velocity_rad_s);
  /* 邮箱忙就跳过本次；仅提交硬件发送请求，不等待总线发送完成。 */
  (void)CanTransport_Send(CAN_APP_FEEDBACK_BASE_ID + motor_id, data);
}
