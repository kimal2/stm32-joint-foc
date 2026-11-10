/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用 PID 控制器（SimpleFOC 2.4 PIDController 的 C 语言移植）
  *
  * 功能：
  *   - Tustin 梯形积分
  *   - 积分和输出限幅，抑制积分饱和
  *   - 可选的输出变化率限制
  *   - 由调用者提供 dt，兼容固定周期与可变周期控制
  *
  * 示例：
  *     static PID_t pid;
  *     PID_Init(&pid, P, I, D, ramp, limit);
  *     u = PID_Update(&pid, reference - feedback, dt);
  ******************************************************************************
  */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float P;
  float I;
  float D;
  float output_ramp;    /* 最大输出变化率（单位/秒），0 表示禁用   */
  float limit;          /* 积分与输出的对称限幅值，<=0 表示禁用 */
  float integral_prev;  /* 内部状态                                   */
  float error_prev;     /* 内部状态                                   */
  float output_prev;    /* 内部状态                                   */
} PID_t;

void  PID_Init(PID_t *pid, float P, float I, float D, float ramp, float limit);
void  PID_Reset(PID_t *pid);
void  PID_SetGains(PID_t *pid, float P, float I, float D);
void  PID_SetLimit(PID_t *pid, float limit);

/* 返回本次采样对应的控制器输出，dt 的单位为秒。 */
float PID_Update(PID_t *pid, float error, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
