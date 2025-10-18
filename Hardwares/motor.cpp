#include "motor.h"

using namespace Motor;



Motor::dc_motor::dc_motor(TIM_HandleTypeDef &motor_tim, uint32_t motor_channel, GPIO_TypeDef *dir_port, uint16_t dir_pin,  int _dir)
	: motor_pwm_tim(motor_tim), motor_pwm_channel(motor_channel), motor_dir_port(dir_port), motor_dir_pin(dir_pin)
{
	dir = _dir;
}

void Motor::dc_motor::motor_output(int pwm)
{
	if (pwm >= 4 && pwm <= 100)
	{
		if (dir == 1)
		{

			HAL_GPIO_WritePin(motor_dir_port, motor_dir_pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(motor_dir_port, motor_dir_pin, GPIO_PIN_SET);
		}
	}
	else if (pwm < -4 && pwm >= -100)
	{
		pwm = -pwm;
		if (dir == 1)
		{
			HAL_GPIO_WritePin(motor_dir_port, motor_dir_pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(motor_dir_port, motor_dir_pin, GPIO_PIN_RESET);
		}
	}
	else
	{
		pwm = 0;
	}

	__HAL_TIM_SET_COMPARE(&motor_pwm_tim, motor_pwm_channel, pwm);
}

void Motor::dc_motor::motor_close_vel(float _target_speed)
{
	target_wheel_speed = _target_speed;
}

void Motor::dc_motor::motor_init(void)
{
//	HAL_TIM_Base_Start_IT(&htim13); // 读取编码器
	HAL_TIM_Base_Start_IT(&htim12); // 计算pid
	HAL_TIM_PWM_Start(&motor_pwm_tim, motor_pwm_channel);
//	HAL_TIM_Encoder_Start(&motor_encoder_tim, TIM_CHANNEL_ALL);
	PID_struct_init(&vel_pid, POSITION_PID, 90, 70,13, 0.009, 0.11);//右边电机这个参数没问题

}

void Motor::dc_motor::get_motor_speed()
{
//	float speed;
//	encoder_get = __HAL_TIM_GET_COUNTER(&motor_encoder_tim);
//	if (encoder_get > 32767)
//	{
//		encoder_get -= 65535;
//	}
//	__HAL_TIM_SET_COUNTER(&motor_encoder_tim, 0);
//	speed = ((((encoder_get*1.00f*2*PI)/encoder_ppr)* (wheel_diameter/2.0)) / (Reduction_ratio * 1000))/(0.01) ; //单位m/s
//	current_wheel_speed = speed;
}