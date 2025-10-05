#include "interrupt.h"
extern Motor::dc_motor motorl;
extern Motor::dc_motor motorr;
extern int ccd_pos;

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
char tx_buff[4];




	//10ms
motorl.get_motor_speed();
motorr.get_motor_speed();	
}
if(htim==&htim12)
{
motorl.motor_output(pid_calc(&motorl.vel_pid,motorl.current_wheel_speed,motorl.target_wheel_speed));
	motorr.motor_output(pid_calc(&motorr.vel_pid,motorr.current_wheel_speed,motorr.target_wheel_speed));
}

}