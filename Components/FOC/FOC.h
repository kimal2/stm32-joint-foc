#ifndef FOC_H
#define FOC_H

#include "foc_utils.h"

/* 保留现有模式接口；d/q 电流闭环由 ADC 中断执行。 */
typedef enum
{
    Type_voltage,
    Type_dc_current,
    Type_foc_current
} TorqueControlType;

extern TorqueControlType torque_controller;

/******************************************************************************/
/**
 *  旋转方向枚举
 */
typedef enum
{
    CW      = 1,  //顺时针
    CCW     = -1, // 逆时针
    UNKNOWN = 0   //方向未知或状态无效
} Direction;

/******************************************************************************/
extern long sensor_direction;
extern float voltage_power_supply;
extern float voltage_limit;
extern float voltage_sensor_align;
extern int  pole_pairs;
/******************************************************************************/
extern float shaft_angle;//!< 当前电机机械角度
extern float electrical_angle;
extern float current_sp;
extern DQVoltage_s voltage;
extern DQCurrent_s current;

extern float sensor_offset;
extern float zero_electric_angle;
/******************************************************************************/
float shaftAngle(void);
float electricalAngle(void);
/******************************************************************************/

void Motor_init(void);
void Motor_initFOC(void);
void loopFOC(void);
void setTargetVotage(float new_target);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
/******************************************************************************/

#endif
