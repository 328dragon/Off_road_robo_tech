#include "state_control.h"
#include "pid.h"
extern float speed_normal;
extern float pwm_l;
extern float pwm_r;
extern int turn_rectngle_flag;
extern pid_t revise_ccd_pid;

extern state_ctrl_t imu_pitch_state_Ctr;
extern state_ctrl_t gray_state_Ctr;
extern state_ctrl_t SR04_state_Ctr;
extern state_ctrl_t imu_yaw_state_Ctr;
extern GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor;
extern GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor_right;

__IO int imu_priority = 1;
__IO int loop_times= 0;
__IO int half_loop=-1;
__IO int stair_flag=0;
__IO int normal_time_cnt = -1;
int turn_times = 0;
extern __IO int stop_flag;
extern __IO int start_flag;
__IO int exist_turn_rectangle_flag=0;

int up_true_slope=0;
int SR04_switch=0;

int pattern_switch_flag=0;

// 陀螺仪
extern float pitch_true;
extern float roll_true;
extern float yaw_true;

void H30_yaw_state_ctrl(state_ctrl_t *_ctr_t, float *cin)
{
    switch (_ctr_t->state_Order)
    {
        // 初状态,接近0度
    case 0:
    {
        // 走过一个pI的时候
        if ((*cin > 170 && *cin <=179 )||(*cin < -170 && *cin > -180))
        {
            _ctr_t->state_Order = 1;
						half_loop=1;		
        }
        break;
    }
    case 1:
    {		
        if ( *cin > -5 && *cin < 5)
        {		
            vTaskDelay(200);
						loop_times++;
						imu_priority = 1;
						half_loop=0;
            _ctr_t->state_Order = 0;
            imu_pitch_state_Ctr.state_Order = 0;
        }
        break;
    }
    default:
        break;
    }
}

//超声波启动
void SR04_state_Controller(state_ctrl_t *_ctr_t, float *cin)
{		
	switch (_ctr_t->state_Order)
    {
    case 0:
    {
        if (*cin < 50.0)
        {
						SR04_switch++;
						vTaskDelay(100);
						if(SR04_switch>=5)
						{
							_ctr_t->state_Order = 1;
						}
        }
        break;
    }
    case 1:
    {		
        if (*cin > 50)
        {		
						SR04_switch--;
					  vTaskDelay(100);
						if(SR04_switch<=0)
						{
							start_flag = 1;
							_ctr_t->state_Order = -1;
						}
        }
        break;
    }
    default:
        break;
    }
}

void H30_pitch_state_ctrl(state_ctrl_t *_ctr_t, float *cin)
{
    switch (_ctr_t->state_Order)
    {
    case 0:
    {
        if (*cin < -12.0)
        {
            imu_priority = 1;
						up_true_slope++;
					if(up_true_slope>2){
							up_true_slope=0;
            _ctr_t->state_Order = 1;}
        }
        break;
    }
    case 1:
    {
			
        if (*cin < 8.0 && *cin > -12.0)
        {
            _ctr_t->state_Order = 2;
        }
        break;
    }
    case 2:
    {
        if (*cin > 8.0)
        {
            vTaskDelay(2000);
						imu_priority = 0;
            _ctr_t->state_Order = -1;
						
        }
        break;
    }
    default:
        break;
    }
}

// 内圈感为状态机
void gw_state_Ctrl(state_ctrl_t *_ctr_t, float *cin)
{

    switch (_ctr_t->state_Order)
    {
    // 状态0加速
    case 0:
    {
        int white_Cnt = 0;
        for (int i = 0; i < 7; i++)
        {
            if (_ctr_t->data[i] == 0)
                white_Cnt++;
        }
				if (white_Cnt >= 5)
            _ctr_t->state_Order = 1;
        break;
    }

    // 静止
    case 1:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 2;
				
        break;
    }
    // 状态2旋转
    case 2:
    {
	
        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 3;
        }
        break;
    }
    // 直线慢速
    case 3:
    {
				exist_turn_rectangle_flag=1;
        if (_ctr_t->data[1] == 0 && _ctr_t->data[2] == 0 && _ctr_t->data[3] == 0 && _ctr_t->data[4] == 0 && _ctr_t->data[5] == 0 && _ctr_t->data[6] == 0)
            _ctr_t->state_Order = 4;
        break;
    }
        // 停下等待
    case 4:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 5;
        break;
    }
    // 旋转
    case 5:
    {
        if ( Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1 && Gw_GrayscaleSensor.data[6] == 1 && Gw_GrayscaleSensor.data[7] == 0)
        {
            _ctr_t->state_Order = 6;
        }
        break;
    }
		case 6:
		{
				vTaskDelay(1500);
				_ctr_t->state_Order = 7;
		}
		case 7:
		{
				vTaskDelay(3200);
				_ctr_t->state_Order = 8;
		}
		//阶梯坡前减速
    case 8:
    {
        int white_Cnt = 0;
        for (int i = 0; i < 7; i++)
        {
            if (_ctr_t->data[i] == 0)
                white_Cnt++;
        }
        if (white_Cnt >= 5)
            _ctr_t->state_Order = 9;
        break;
    }

    // 静止
    case 9:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 10;

        break;
    }
    // 状态2旋转
    case 10:
    {

        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 11;
        }
        break;
    }
    // 直线慢速
    case 11:
    {
        if (_ctr_t->data[1] == 0 && _ctr_t->data[2] == 0 && _ctr_t->data[3] == 0 && _ctr_t->data[4] == 0 && _ctr_t->data[5] == 0 && _ctr_t->data[6] == 0)
            _ctr_t->state_Order = 12;
        break;
    }
    // 停下等待1s
    case 12:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 13;
        break;
    }

    // 旋转
    case 13:
    {
        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 14;
         
        }
        break;
    }
		case 14:
    {
				vTaskDelay(1500);
        _ctr_t->state_Order = 0;
        break;
    }

    default:
        break;
    }
}

void car_speed_state_switch(car_state *car)
{
    switch (car->_car_mode)
    {
    case begin_state:
    {
        car->update_vel_flag = 1;
				pid_reset(&revise_ccd_pid,0.01,0.01,0.03);//normal_speed=1.6时pid参数
				speed_normal = 1.6;
        car->vel_Control();
        break;
    }
    case normal:
    {		
				car->update_vel_flag = 1;
				pid_reset(&revise_ccd_pid,0.03,0.03,0.01);//normal_speed=1.2时pid参数
				speed_normal = 1.3;
        car->vel_Control();
        break;
    }
    case speed_up:
    {
        car->update_vel_flag = 1;
				pid_reset(&revise_ccd_pid,0.01,0,0.03);//normal_speed=1.6时pid参数
        speed_normal = 1.8;
        car->vel_Control();
        break;
    }
		case speed_up_straight:
    {
        car->update_vel_flag = 1;
				pid_reset(&revise_ccd_pid,0.01,0,0.03);//normal_speed=1.8时pid参数
        speed_normal = 1.6;
        car->vel_Control();
        break;
    }
    case slow_down:
    {
        car->update_vel_flag = 1;
				pid_reset(&revise_ccd_pid,0.02,0,0.005);//normal_speed=1时pid参数
        speed_normal = 1;
        car->vel_Control();
        break;
    }
		case slow_down_slope:
    {
        car->update_vel_flag = 1;
				pid_reset(&revise_ccd_pid,0.015,0,0.005);//normal_speed=1时pid参数
        speed_normal = 0.8;
        car->vel_Control();
        break;
    }
    case turn_state:
    {
        car->update_vel_flag = 0;
        pwm_l = -9;
        pwm_r = 15;
        break;
    }

    case turn_state_second:
    {
        car->update_vel_flag = 0;
        pwm_l = -9;
        pwm_r = 15;
        break;
    }
    case stop_car:
    {
        car->update_vel_flag = 0;
        pwm_l = 0;
        pwm_r = 0;
        break;
    }
    default:
        break;
    }
}

//有障碍
void car_patter_Switch2(car_state *car)
{
				if(loop_times==3)
				{
					vTaskDelay(1500);
					stop_flag=1;
				}
				
        if (imu_priority == 1)
        {
            if (imu_pitch_state_Ctr.state_Order == 0)
            {		
								//第二圈开始回正前减速一段时间
								if(loop_times!=0)
								{
										car->_car_mode = normal;
										vTaskDelay(1500);
								}
                car->_car_mode = begin_state;
            }
            else if (imu_pitch_state_Ctr.state_Order == 1)
            {
                car->_car_mode = speed_up;
            }
            else if (imu_pitch_state_Ctr.state_Order == 2)
            {
                car->_car_mode = slow_down_slope;
            }
        }
        else if (imu_priority == 0)
        {		
						//转完半圈可以判断是否在内圈
						if(half_loop==1&&exist_turn_rectangle_flag==0)
						{			
								gray_state_Ctr.state_Order = -1;
						}
						//转过第一个弯后减速一下
						if (gray_state_Ctr.state_Order == -1&&half_loop==1)
						{
								car->_car_mode = normal;
								vTaskDelay(1000);
								half_loop=-1;
						}	
						if (gray_state_Ctr.state_Order == -1&&half_loop==-1)
								car->_car_mode = speed_up_straight;
		
						if (gray_state_Ctr.state_Order == 0)
						{		
								//下坡后降点速
								car->_car_mode = normal;
								vTaskDelay(3000);
								//转直角弯前需要减速
								car->_car_mode = slow_down;
						}
						
            else if (gray_state_Ctr.state_Order == 1)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 2)
            {
                car->_car_mode = turn_state;
            }
            else if (gray_state_Ctr.state_Order == 3)
            {
                car->_car_mode = slow_down;
            }
            else if (gray_state_Ctr.state_Order == 4)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 5)
            {
                car->_car_mode = turn_state;           
            }
						else if (gray_state_Ctr.state_Order == 6)
            {
                car->_car_mode = normal;
            }
            else if (gray_state_Ctr.state_Order == 7)
            {
                car->_car_mode = speed_up_straight;
            }
						else if (gray_state_Ctr.state_Order == 8)
            {
                car->_car_mode = slow_down;
            }
            else if (gray_state_Ctr.state_Order == 9)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 10)
            {
                car->_car_mode = turn_state;
            }
            else if (gray_state_Ctr.state_Order == 11)
            {
                car->_car_mode = slow_down;
            }
            else if (gray_state_Ctr.state_Order == 12)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 13)
            {
                car->_car_mode = turn_state;
            }
						else if (gray_state_Ctr.state_Order == 14)
            {
                car->_car_mode = normal;
            }
        }
}

//无障碍
void car_patter_Switch1(car_state *car)
{
		if(loop_times==3)
		{
			vTaskDelay(1500);
			stop_flag=1;
		}
		if(half_loop==1&&exist_turn_rectangle_flag==0)
		{			
				gray_state_Ctr.state_Order = -1;
		}
		
		if (gray_state_Ctr.state_Order == -1&&half_loop==1)
		{
				car->_car_mode = normal;
				vTaskDelay(1000);
				half_loop=-1;
		}
		if (gray_state_Ctr.state_Order == -1&&half_loop==0)
		{
				car->_car_mode = normal;
				vTaskDelay(1000);
				half_loop=-1;
		}
		if (gray_state_Ctr.state_Order == -1&&half_loop==-1)
        car->_car_mode = speed_up_straight;
		
    if (gray_state_Ctr.state_Order == 0)
		{		
				car->_car_mode = speed_up_straight;
				vTaskDelay(3000);
				car->_car_mode = slow_down;
		}
    				
            else if (gray_state_Ctr.state_Order == 1)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 2)
            {
                car->_car_mode = turn_state;
            }
            else if (gray_state_Ctr.state_Order == 3)
            {
                car->_car_mode = slow_down;
            }
            else if (gray_state_Ctr.state_Order == 4)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 5)
            {
                car->_car_mode = turn_state;           
            }
						else if (gray_state_Ctr.state_Order == 6)
            {
                car->_car_mode = normal;
            }
            else if (gray_state_Ctr.state_Order == 7)
            {
                car->_car_mode = speed_up_straight;
            }
						else if (gray_state_Ctr.state_Order == 8)
            {
                car->_car_mode = slow_down;
            }
            else if (gray_state_Ctr.state_Order == 9)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 10)
            {
                car->_car_mode = turn_state;
            }
            else if (gray_state_Ctr.state_Order == 11)
            {
                car->_car_mode = slow_down;
            }
            else if (gray_state_Ctr.state_Order == 12)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 13)
            {
                car->_car_mode = turn_state;
            }
						else if (gray_state_Ctr.state_Order == 14)
            {
                car->_car_mode = normal;
            }
}
