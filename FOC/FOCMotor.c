/************************************************
本文件主要来自CSDN博主loop222对SimpleFOC项目的移植
原教程：https://blog.csdn.net/loop222/article/details/120471390
************************************************/

#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "MagneticSensor.h" 

/******************************************************************************/
float shaft_angle;//!< 当前电机机械角度
float electrical_angle;
float current_sp;
DQVoltage_s voltage;
DQCurrent_s current;

TorqueControlType torque_controller;

float sensor_offset=0;
float zero_electric_angle;
/******************************************************************************/
// 计算机械轴角度
float shaftAngle(void)
{
  return sensor_direction*getAngle() - sensor_offset;
}
/******************************************************************************/
float electricalAngle(void)
{
  return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}
/******************************************************************************/


