#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H

/******************************************************************************/
extern long  cpr;
extern float full_rotation_offset;
extern long angle_data, angle_data_prev;
extern unsigned long velocity_calc_timestamp;
extern float angle_prev;
/* 上报角度单位为 rad，采用连续机械角度，方向和零位与 shaft_angle 一致。
 * 只供状态上报或 Ozone 观察，不参与 FOC 换相。
 */
extern volatile float encoder_angle_unfiltered_rad;
extern volatile float encoder_angle_report_rad;
/******************************************************************************/
void MagneticSensor_Init(void);
float getAngle(void);
float getVelocity(void);
void MagneticSensor_UpdateReportAngle(float mechanical_angle);
/******************************************************************************/

#endif
