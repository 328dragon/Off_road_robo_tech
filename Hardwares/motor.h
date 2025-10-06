#ifndef __MOTOR_H
#define __MOTOR_H
extern "C" {
#include "main.h"
#include "tim.h"
#include "pid.h"
};
#define PI 3.1415926
 namespace  Motor{
//单个直流普通电机
class dc_motor
{
private:
//电机硬件（电路）特性及其初始化
TIM_HandleTypeDef &motor_pwm_tim;//电机PWM定时器
uint32_t motor_pwm_channel;
GPIO_TypeDef* motor_dir_port;//电机方向控制引脚端口
uint16_t motor_dir_pin;
TIM_HandleTypeDef &motor_encoder_tim;//编码器定时器
int wheel_diameter=104; //轮子B直径，单位mm
int Reduction_ratio=44;//减速比s
int encoder_ppr=64; //编码器每转脉冲数
//电机控制参数
int dir;        //电机方向
int out_pwm;        //电机PWM占空比

int encoder_get;


public:
	float current_wheel_speed;   //当前轮子速度，单位mm/s
float target_wheel_speed;
pid_t vel_pid;
    dc_motor(TIM_HandleTypeDef &motor_tim, uint32_t motor_channel, GPIO_TypeDef* dir_port, uint16_t dir_pin, TIM_HandleTypeDef &encoder_tim,int _dir);
    void motor_init(void);
		void motor_close_vel(float _target_speed);
    void motor_output(int pwm);
    void get_motor_speed();
};

}

#endif
