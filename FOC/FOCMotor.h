#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "foc_utils.h" 
/******************************************************************************/
/**
 *  转矩控制类型
 */
typedef enum
{
	Type_voltage, //!< 通过电压控制转矩
	Type_dc_current, //!< 通过直流电流幅值控制转矩
	Type_foc_current //!< 通过 d/q 轴电流控制转矩
} TorqueControlType;

extern TorqueControlType torque_controller;
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

#endif

