#ifndef CAN_APP_H
#define CAN_APP_H

#include "CanTransport.h"

#define CAN_APP_CURRENT_ID 0x080U
#define CAN_APP_FEEDBACK_BASE_ID 0x100U

extern volatile uint8_t motor_id;
extern volatile uint32_t can_command_rejected_count;

/* 电机初始化后启动 CAN。优先读取 Flash ID，无有效 ID 时使用默认 ID。 */
HAL_StatusTypeDef CanApp_Init(void);
/* 接收处理：主循环取出最新接收缓存并解析，不发送反馈。 */
void CanApp_ProcessReceivedCommands(void);
/* 接收解析：检查电流帧，按本机 ID 解码目标电流并应用。 */
HAL_StatusTypeDef CanApp_ParseCurrentCommand(const CanFrame *frame);
/* 状态上报：SysTick 每 1 ms 调用，只打包已有缓存并提交 CAN 发送，不阻塞等待。 */
void CanApp_SendMotorFeedback(void);
/* 反馈编码：角度 0.001 rad/LSB、速度 0.01 rad/s/LSB，大端；最后两字节为零。 */
void CanApp_PackMotorFeedback(uint8_t data[8], float angle_rad, float velocity_rad_s);

#endif
