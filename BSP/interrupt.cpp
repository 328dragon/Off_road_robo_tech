#include "interrupt.h"
extern Motor::dc_motor motorl;
extern Motor::dc_motor motorr;
extern CCD_t front_ccd;
extern __IO int start_flag;
extern __IO int stop_flag;
extern car_state car;
extern SR04_t SR04_front;

float pwm_l=0;
float pwm_r=0;
#define vel_convert (20/1.5)

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance==TIM11)
{
  SR04_Elapsed_callback(&SR04_front);
}


if(htim==&htim13)
{
//10ms
motorl.get_motor_speed();
motorr.get_motor_speed();	
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
//motorl.motor_output(pid_calc(&motorl.vel_pid,motorl.current_wheel_speed,motorl.target_wheel_speed));
//motorr.motor_output(pid_calc(&motorr.vel_pid,motorr.current_wheel_speed,motorr.target_wheel_speed));
	if(car.update_vel_flag==1)
	{
	pwm_l = car.target_speedl*vel_convert;
	pwm_r = car.target_speedr*vel_convert;
	}

	motorl.motor_output(pwm_l);
	motorr.motor_output(pwm_r);
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
if(htim->Instance==TIM5)
{
  SR04_Echo_IC_callback(&SR04_front);
}

}