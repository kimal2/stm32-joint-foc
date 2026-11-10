/**
  ******************************************************************************
  * @file    pid.c
  * @brief   通用 PID 控制器（SimpleFOC 2.4 PIDController 的 C 语言移植）
  ******************************************************************************
  */

#include "pid.h"
#include "foc_utils.h"

void PID_Init(PID_t *pid, float P, float I, float D, float ramp, float limit)
{
  pid->P = P;
  pid->I = I;
  pid->D = D;
  pid->output_ramp = ramp;
  pid->limit = limit;
  PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
  pid->integral_prev = 0.0f;
  pid->error_prev = 0.0f;
  pid->output_prev = 0.0f;
}

void PID_SetGains(PID_t *pid, float P, float I, float D)
{
  pid->P = P;
  pid->I = I;
  pid->D = D;
}

void PID_SetLimit(PID_t *pid, float limit)
{
  pid->limit = limit;
}

float PID_Update(PID_t *pid, float error, float dt)
{
  float proportional;
  float integral;
  float derivative;
  float output;

  if (dt <= 0.0f)
  {
    return pid->output_prev;
  }

  proportional = pid->P * error;

  /* Tustin 梯形积分：u_ik = u_ik_1 + I*dt/2*(ek + ek_1) */
  integral = pid->integral_prev +
             pid->I * dt * 0.5f * (error + pid->error_prev);
  if (pid->limit > 0.0f)
  {
    integral = _constrain(integral, -pid->limit, pid->limit);
  }

  derivative = pid->D * (error - pid->error_prev) / dt;

  output = proportional + integral + derivative;
  if (pid->limit > 0.0f)
  {
    output = _constrain(output, -pid->limit, pid->limit);
  }

  /* 可选的输出变化率限制 */
  if (pid->output_ramp > 0.0f)
  {
    float rate = (output - pid->output_prev) / dt;

    if (rate > pid->output_ramp)
    {
      output = pid->output_prev + pid->output_ramp * dt;
    }
    else if (rate < -pid->output_ramp)
    {
      output = pid->output_prev - pid->output_ramp * dt;
    }
  }

  pid->integral_prev = integral;
  pid->error_prev = error;
  pid->output_prev = output;

  return output;
}
