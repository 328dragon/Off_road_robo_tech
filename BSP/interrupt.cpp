#include "interrupt.h"
extern Motor::dc_motor motorl;
extern Motor::dc_motor motorr;
extern CCD_t front_ccd;
int stop_flag=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		if ((__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)) 
		{
			atk_ms53l0m.data_receive();
		}

		if ((__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET))
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart3);
			if(atk_ms53l0m.rx_len != 0 && atk_ms53l0m.rx_ok == 0)
			{	
				atk_ms53l0m.rx_ok = 1;
			}
		}
	}
		
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim==&htim13)
{
	//10ms
motorl.get_motor_speed();
motorr.get_motor_speed();	
}
if(htim==&htim12)
{
	if(stop_flag==0)
	{
	motorl.motor_output(pid_calc(&motorl.vel_pid,motorl.current_wheel_speed,motorl.target_wheel_speed));
motorr.motor_output(pid_calc(&motorr.vel_pid,motorr.current_wheel_speed,motorr.target_wheel_speed));
	}
	else if(stop_flag==1)
	{
		motorl.motor_output(0);
motorr.motor_output(0);
	
	}

}

}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_2)
	{
	stop_flag=1;
	}
ccd_exti_callback(&front_ccd,GPIO_Pin);

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
ccd_spi_rx_cplt_callback(&front_ccd,hspi);
	
}