#ifndef BLDCMotor_H
#define BLDCMotor_H

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
void Motor_init(void);
void Motor_initFOC(void);
void loopFOC(void);
void setTargetVotage(float new_target);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
/******************************************************************************/

#endif
