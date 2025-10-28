#include "interrupt.h"
extern Motor::dc_motor motorl;
extern Motor::dc_motor motorr;
extern CCD_t front_ccd;
extern __IO int start_flag;
extern __IO int stop_flag;
extern car_state car;
extern SR04_t front_sr04;

float pwm_l=0;
float pwm_r=0;
#define vel_convert (20/1.5)

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{

//		
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim==&htim11)
{
 SR04_Elapsed_callback(&front_sr04);
}

if(htim==&htim12)
{
if(stop_flag==1)
{
motorl.motor_output(0);
motorr.motor_output(0);
}	
else if(stop_flag==0&&start_flag==1)
{
	if(car.update_vel_flag==1)
	{
	pwm_l = car.target_speedl*vel_convert;
	pwm_r = car.target_speedr*vel_convert;
	}

	motorl.motor_output(pwm_l);
	motorr.motor_output(pwm_r);
}
}
static int led_flag=0;
if(htim==&htim13)
{	
	if(stop_flag!=1)
	{
		led_flag++;
	if(led_flag==500)
	{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
	}
	else if(led_flag==1000)
	{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
	}

	else if(led_flag>1000)
		led_flag=0;
	}
	else if(stop_flag==1)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
	}
}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

ccd_exti_callback(&front_ccd,GPIO_Pin);

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
ccd_spi_rx_cplt_callback(&front_ccd,hspi);
	
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
if(htim==front_sr04.Echo_htim)
{
  SR04_Echo_IC_callback(&front_sr04);
}
}