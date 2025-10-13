#include "HCSR04.h"

uint16_t Time = 0;

///////////////////超声波测距函数/////////////////////
void HCSR04_Start(void)																										//手动中断，启用一次超声波模块
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port GPIOB, TRIG_Pin,GPIO_PIN_SET);
	Delayus(20);
	HAL_GPIO_WritePin(TRIG_GPIO_Port GPIOB, TRIG_Pin,GPIO_PIN_RESET);
}

uint16_t HCSR04_GetValue(void)																						//启用超声波模块，得到超声波模块工作一次的定时器计数值
{
  HCSR04_Start();
  Delayms(20);
	return Time;
}

float HCSR04_Length(void)																									//根据定时器计数值计算与前方物体距离（单位：厘米）
{
	float length = HCSR04_GetValue()*17*0.001;
	return length;
}