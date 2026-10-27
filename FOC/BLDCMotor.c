/************************************************
本文件主要来自CSDN博主loop222对SimpleFOC项目的移植
原教程：https://blog.csdn.net/loop222/article/details/120471390
本项目删除了部分代码，添加了Flash数据读取和对蜂鸣功能的兼容
************************************************/

#include "BLDCMotor.h"
#include "FOCMotor.h"
#include "foc_utils.h"
#include "gpio.h"
#include <stdio.h>
#include "MagneticSensor.h"
#include "tim.h"
#include "FlashStorage.h"

#define M1_Enable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define M1_Disable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define MOTOR_POLE_PAIRS 14   /* 4310 motor */

/*
 * 1 = always run sensor alignment at boot and refresh the Flash cache.
 * Set to 0 once the calibration is known-good to boot faster.
 */
#define FORCE_SENSOR_ALIGN 0

extern float target;

long sensor_direction;
float voltage_power_supply;
float voltage_limit;
float voltage_sensor_align;
int  pole_pairs;
unsigned long open_loop_timestamp;
float velocity_limit;

int alignSensor(void);
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
/******************************************************************************/
void Motor_init(void)
{
	printf("MOT: Init\r\n");

	if(voltage_sensor_align > voltage_limit)
		voltage_sensor_align = voltage_limit;

	pole_pairs=MOTOR_POLE_PAIRS;
	sensor_direction=UNKNOWN;
	M1_Enable;
	printf("MOT: Enable driver.\r\n");
}
/******************************************************************************/
void Motor_initFOC(void)
{
#if FORCE_SENSOR_ALIGN
	if(alignSensor()) //开机自动校准：检测传感器方向、极对数和电角度零点
		Flash_SaveMotorParam(pole_pairs, zero_electric_angle, sensor_direction);
#else
	// 读取Flash中的校准数据；若极对数与当前电机不符则重新校准
	if((Flash_ReadMotorParam(&pole_pairs, &zero_electric_angle, (int*)&sensor_direction) == -1) ||
	   (pole_pairs != MOTOR_POLE_PAIRS))
	{
		if(alignSensor())
			Flash_SaveMotorParam(pole_pairs, zero_electric_angle, sensor_direction);
	}
#endif

	//added the shaft_angle update
	angle_prev=getAngle();  //getVelocity(),make sure velocity=0 after power on
	HAL_Delay(5);
	shaft_angle = shaftAngle();// shaft angle

	HAL_Delay(200);
}
/******************************************************************************/
int alignSensor(void)
{
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;

	printf("MOT: Align sensor.\r\n");

	// find natural direction
	// move one electrical revolution forward
	for(i=0; i<=500; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(voltage_sensor_align, 0,  angle);
		HAL_Delay(2);
	}
	mid_angle=getAngle();

	for(i=500; i>=0; i--)
	{
		angle = _3PI_2 + _2PI * i / 500.0 ;
		setPhaseVoltage(voltage_sensor_align, 0,  angle);
		HAL_Delay(2);
	}
	end_angle=getAngle();
	setPhaseVoltage(0, 0, 0);
	HAL_Delay(200);

	printf("mid_angle=%.4f\r\n",mid_angle);
	printf("end_angle=%.4f\r\n",end_angle);

	moved =  fabs(mid_angle - end_angle);
	if((mid_angle == end_angle)||(moved < 0.02))  //相等或者几乎没有动
	{
		printf("MOT: Failed to notice movement loop222.\r\n");
		M1_Disable;    //电机检测不正常，关闭驱动
		return 0;
	}
	else if(mid_angle < end_angle)
	{
		printf("MOT: sensor_direction==CCW\r\n");
		sensor_direction=CCW;
	}
	else
	{
		printf("MOT: sensor_direction==CW\r\n");
		sensor_direction=CW;
	}


	printf("MOT: PP check: ");    //计算Pole_Pairs
	if( fabs(moved*pole_pairs - _2PI) > 0.5 )  // 0.5 is arbitrary number it can be lower or higher!
	{
		printf("fail - estimated pp:");
		pole_pairs=_2PI/moved+0.5;     //浮点数转整形，四舍五入
		printf("%d\r\n",pole_pairs);
  }
	else
		printf("OK!\r\n");


	setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);  //计算零点偏移角度
	HAL_Delay(700);
	zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), pole_pairs));
	HAL_Delay(20);
	printf("MOT: Zero elec. angle:");
	printf("%.4f\r\n",zero_electric_angle);

	setPhaseVoltage(0, 0, 0);
	HAL_Delay(200);

	return 1;
}
/******************************************************************************/
void loopFOC(void)
{
	if( controller==Type_angle_openloop || controller==Type_velocity_openloop ) return;

	shaft_angle = shaftAngle();// shaft angle
	electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first

	// d/q current mode: the PWM is driven from the ADC interrupt (CurrentControl.c)
	if(torque_controller == Type_foc_current) return;

	switch(torque_controller)
	{
		case Type_voltage:  // no need to do anything really
			break;
		case Type_dc_current:
			break;
		case Type_foc_current:
			break;
		default:
			printf("MOT: no torque control selected!");
			break;
	}
	// set the phase voltage - FOC heart function :)
	extern volatile uint8_t beepPlaying;
	if(beepPlaying == 0) //没在蜂鸣状态则正常输出电压
		setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}
/******************************************************************************/
void setTargetVotage(float new_target)
{
	voltage.q = new_target;
}
/******************************************************************************/
/*
 * Phase voltage in volts -> timer compare value.
 * Same as SimpleFOC's driver->setPwm(): duty = U / Vbus, clamped to [0, 1].
 */
static uint32_t phaseVoltageToCCR(float U, uint32_t pwm_period)
{
	if(U <= 0.0f)
		return 0U;
	if(U >= voltage_power_supply)
		return pwm_period;
	return (uint32_t)(U * (float)pwm_period / voltage_power_supply);
}
/******************************************************************************/
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float _sa,_ca;
	float Ualpha,Ubeta;
	float Ua,Ub,Uc;
	float Umin,Umax;
	float center;
	uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;

	// _sin/_cos are table based and need the angle in [0, 2PI]
	angle_el = _normalizeAngle(angle_el);

	_sincos(angle_el, &_sa, &_ca);

	// inverse Park transform
	Ualpha = _ca * Ud - _sa * Uq;
	Ubeta  = _sa * Ud + _ca * Uq;

	// Clarke transform (amplitude invariant)
	Ua = Ualpha;
	Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
	Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

	// centered SVPWM: midpoint clamp (zero sequence injection)
	Umin = (Ua < Ub) ? ((Ua < Uc) ? Ua : Uc) : ((Ub < Uc) ? Ub : Uc);
	Umax = (Ua > Ub) ? ((Ua > Uc) ? Ua : Uc) : ((Ub > Uc) ? Ub : Uc);
	center = 0.5f * voltage_power_supply - 0.5f * (Umax + Umin);

	Ua += center;
	Ub += center;
	Uc += center;

	// calculate the duty cycles
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, phaseVoltageToCCR(Ua, pwm_period));
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, phaseVoltageToCCR(Ub, pwm_period));
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, phaseVoltageToCCR(Uc, pwm_period));
}
