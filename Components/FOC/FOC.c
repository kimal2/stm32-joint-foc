/************************************************
本文件主要来自CSDN博主loop222对SimpleFOC项目的移植
原教程：https://blog.csdn.net/loop222/article/details/120471390
本项目删除了部分代码，添加了Flash数据读取和对蜂鸣功能的兼容
************************************************/

#include "FOC.h"
#include "foc_utils.h"
#include "gpio.h"
#include "MagneticSensor.h"
#include "tim.h"
#include "FlashStorage.h"

#define M1_Enable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define M1_Disable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define MOTOR_POLE_PAIRS 14   /* 4310 电机 */

/*
 * 1 = 每次启动均执行传感器校准，并更新 Flash 中的校准数据。
 * 确认校准数据有效后可设为 0，以加快启动速度。
 */
#define FORCE_SENSOR_ALIGN 0

extern float target;

long sensor_direction;
float voltage_power_supply;
float voltage_limit;
float voltage_sensor_align;
int  pole_pairs;

TorqueControlType torque_controller;

float shaft_angle;//!< 当前电机机械角度
float electrical_angle;
float current_sp;
DQVoltage_s voltage;
DQCurrent_s current;


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

int alignSensor(void);
/******************************************************************************/
void Motor_init(void)
{

	if(voltage_sensor_align > voltage_limit)
		voltage_sensor_align = voltage_limit;

	pole_pairs=MOTOR_POLE_PAIRS;
	sensor_direction=UNKNOWN;
	M1_Enable;
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

	//更新机械轴角度
	angle_prev=getAngle();  //初始化角度历史值
	HAL_Delay(5);
	shaft_angle = shaftAngle();// 机械轴角度

	HAL_Delay(200);
}
/******************************************************************************/
int alignSensor(void)
{
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;


	// 检测传感器的自然旋转方向
	// 正向旋转一个电周期
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


	moved =  fabs(mid_angle - end_angle);
	if((mid_angle == end_angle)||(moved < 0.02))  //相等或者几乎没有动
	{
		M1_Disable;    //电机检测不正常，关闭驱动
		return 0;
	}
	else if(mid_angle < end_angle)
	{
		sensor_direction=CCW;
	}
	else
	{
		sensor_direction=CW;
	}


	if( fabs(moved*pole_pairs - _2PI) > 0.5 )  // 0.5 为经验阈值，可按实际情况调整
	{
		pole_pairs=_2PI/moved+0.5;     //浮点数转整形，四舍五入
  }


	setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);  //计算零点偏移角度
	HAL_Delay(700);
	zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), pole_pairs));
	HAL_Delay(20);

	setPhaseVoltage(0, 0, 0);
	HAL_Delay(200);

	return 1;
}
/******************************************************************************/
void loopFOC(void)
{
	shaft_angle = shaftAngle();// 机械轴角度
	electrical_angle = electricalAngle();// 电角度计算前必须先调用 shaftAngle
	MagneticSensor_UpdateReportAngle(shaft_angle); // 仅更新上报角度，不影响换相

	// d/q 电流模式下，由 ADC 中断中的电流环更新 PWM（CurrentControlIQ.c）
	if(torque_controller == Type_foc_current) return;

	// 设置相电压，这是 FOC 输出的核心步骤
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
 * 将相电压（单位 V）换算成定时器比较值。
 * 与 SimpleFOC 的 driver->setPwm() 一致：占空比 = U / Vbus，限幅到 [0, 1]。
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

	// _sin/_cos 使用查表计算，输入角度需归一化到 [0, 2PI]
	angle_el = _normalizeAngle(angle_el);

	_sincos(angle_el, &_sa, &_ca);

	// 逆 Park 变换
	Ualpha = _ca * Ud - _sa * Uq;
	Ubeta  = _sa * Ud + _ca * Uq;

	// Clarke 变换（幅值不变形式）
	Ua = Ualpha;
	Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
	Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

	// 中心对齐 SVPWM：通过零序注入实现中点钳位
	Umin = (Ua < Ub) ? ((Ua < Uc) ? Ua : Uc) : ((Ub < Uc) ? Ub : Uc);
	Umax = (Ua > Ub) ? ((Ua > Uc) ? Ua : Uc) : ((Ub > Uc) ? Ub : Uc);
	center = 0.5f * voltage_power_supply - 0.5f * (Umax + Umin);

	Ua += center;
	Ub += center;
	Uc += center;

	// 计算并设置三相 PWM 占空比
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, phaseVoltageToCCR(Ua, pwm_period));
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, phaseVoltageToCCR(Ub, pwm_period));
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, phaseVoltageToCCR(Uc, pwm_period));
}
