#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/* 一维卡尔曼滤波，状态模型和观测模型均为 1。
 * q 为每次有效采样的过程噪声方差，r 为观测噪声方差，要求 q > 0、r > 0。
 * 角度输入必须先展开为连续角度，不能直接滤波跨越 0/2π 的单圈角度。
 */
typedef struct
{
  float estimate;
  float covariance;
  float gain;
  float q;
  float r;
} KalmanFilter;

/* 用首次有效测量初始化，避免从 0 缓慢收敛造成启动角度偏差。 */
void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial);
float KalmanFilter_Update(KalmanFilter *filter, float measurement);

#endif
