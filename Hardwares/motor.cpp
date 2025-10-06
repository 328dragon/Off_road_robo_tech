#include "motor.h"

using namespace Motor;

Motor::dc_motor motorl(htim8, TIM_CHANNEL_4, PH1_GPIO_Port, PH1_Pin, htim3, 1);
Motor::dc_motor motorr(htim8, TIM_CHANNEL_3, PH2_GPIO_Port, PH2_Pin, htim2, 0);
Motor::dc_motor::dc_motor(TIM_HandleTypeDef &motor_tim, uint32_t motor_channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, TIM_HandleTypeDef &encoder_tim, int _dir)
	: motor_pwm_tim(motor_tim), motor_pwm_channel(motor_channel), motor_dir_port(dir_port), motor_dir_pin(dir_pin), motor_encoder_tim(encoder_tim)
{
	dir = _dir;
}

void Motor::dc_motor::motor_output(int pwm)
{
	if (pwm >= 6 && pwm <= 100)
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
	else if (pwm < -6 && pwm >= -100)
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
	HAL_TIM_Base_Start_IT(&htim13); // 读取编码器
	HAL_TIM_Base_Start_IT(&htim12); // 计算pid
	HAL_TIM_PWM_Start(&motor_pwm_tim, motor_pwm_channel);
	HAL_TIM_Encoder_Start(&motor_encoder_tim, TIM_CHANNEL_ALL);
	PID_struct_init(&vel_pid, POSITION_PID, 90, 70,100, 0.5, 0.05);//这个参数能跑到3m/s左右

	// motor_output(10);
}

void Motor::dc_motor::get_motor_speed()
{
	float speed;
	encoder_get = __HAL_TIM_GET_COUNTER(&motor_encoder_tim);
	if (encoder_get > 32767)
	{
		encoder_get -= 65535;
	}
	__HAL_TIM_SET_COUNTER(&motor_encoder_tim, 0);
		speed = ((((encoder_get*1.00f*2*PI)/encoder_ppr)* (wheel_diameter/2.0)) / (Reduction_ratio * 1000))/(0.01) ; //单位m/s
//		speed =( (encoder_get * PI * wheel_diameter) /
//	        (encoder_ppr * Reduction_ratio * 0.01))/1000;
//	speed = encoder_get;
	//	if(dir==0)
	//	{
	//		speed=-speed;
	//	}
	current_wheel_speed = speed;
}