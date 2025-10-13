#include "HCSR04.h"

uint16_t Time = 0;

///////////////////��������ຯ��/////////////////////
void HCSR04_Start(void)																										//�ֶ��жϣ�����һ�γ�����ģ��
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port GPIOB, TRIG_Pin,GPIO_PIN_SET);
	Delayus(20);
	HAL_GPIO_WritePin(TRIG_GPIO_Port GPIOB, TRIG_Pin,GPIO_PIN_RESET);
}

uint16_t HCSR04_GetValue(void)																						//���ó�����ģ�飬�õ�������ģ�鹤��һ�εĶ�ʱ������ֵ
{
  HCSR04_Start();
  Delayms(20);
	return Time;
}

float HCSR04_Length(void)																									//���ݶ�ʱ������ֵ������ǰ��������루��λ�����ף�
{
	float length = HCSR04_GetValue()*17*0.001;
	return length;
}