#include "KalmanFilter.h"

/* 参考 foc-wheel-legged-robot/software/Src/main.c 中的一维卡尔曼公式，
 * 将固定为 1 的 A、H 以及可复用的中间变量化简。
 */
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial)
{
  filter->estimate = initial;
  filter->covariance = 0.0f;
  filter->gain = 0.0f;
  filter->q = q;
  filter->r = r;
}

float KalmanFilter_Update(KalmanFilter *filter, float measurement)
{
  float predicted_covariance = filter->covariance + filter->q;

  filter->gain = predicted_covariance / (predicted_covariance + filter->r);
  filter->estimate += filter->gain * (measurement - filter->estimate);
  filter->covariance = (1.0f - filter->gain) * predicted_covariance;
  return filter->estimate;
}
