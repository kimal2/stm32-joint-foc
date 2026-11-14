#ifndef CURRENT_CONTROL_CONFIG_H
#define CURRENT_CONTROL_CONFIG_H

/* IQmath 电流环参数，常量在编译时转换为 Q20。 */
#define MOTOR_R_OHM         10.32f
#define MOTOR_L_H           0.00326f
#define CURRENT_BW_HZ       200.0f
#define CURRENT_ADC_DECIMATION 2U       /* 20 kHz ADC 采样二分频，电流环约 10 kHz */
#define CURRENT_TS          0.0001f     /* 电流环周期 100 微秒，用于 PI 和低通滤波 */
#define CURRENT_ULIMIT      12.0f       /* 电压矢量幅值上限，单位 V */
#define CURRENT_LIMIT       1.0f        /* q 轴目标电流上限，单位 A */
#define CURRENT_LPF_TF      0.0001592f
#define CURRENT_LPF_K       (CURRENT_TS / (CURRENT_LPF_TF + CURRENT_TS))
#define CURRENT_KP          (MOTOR_L_H * 6.2831853f * CURRENT_BW_HZ)
#define CURRENT_KI          (MOTOR_R_OHM * 6.2831853f * CURRENT_BW_HZ)

#endif
