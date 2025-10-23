#include "state_control.h"
extern float speed_normal;
extern float pwm_l;
extern float pwm_r;
extern int turn_rectngle_flag;

extern state_ctrl_t imu_pitch_state_Ctr;
extern state_ctrl_t gray_state_Ctr;
extern state_ctrl_t SR04_state_Ctr;
extern state_ctrl_t imu_yaw_state_Ctr;
extern GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor;
extern GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor_right;

int imu_priority = 1;
int loop_times= 0;
int obstacle_cnt = 0;
__IO int normal_time_cnt = -1;
int turn_times = 0;
extern __IO int stop_flag;
__IO int exist_turn_rectangle_flag=0;

int up_true_slope=0;

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
        }
        break;
    }
    case 1:
    {
        if ( *cin > -5 && *cin < 5)
        {
            vTaskDelay(200);
						loop_times++;
            _ctr_t->state_Order = 0;
            imu_priority = 1;
            imu_pitch_state_Ctr.state_Order = 0;
        }
        break;
    }
    default:
        break;
    }
}

void SR04_state_Controller(state_ctrl_t *_ctr_t, float *cin)
{
    switch (_ctr_t->state_Order)
    {
    case 0:
    {
        if (*cin < 20.0)
        {
            _ctr_t->state_Order = 1;
        }
        break;
    }
    // 前方是第一个障碍
    case 1:
    {
        if (*cin > 60)
        {
            obstacle_cnt++;
            vTaskDelay(200);
            _ctr_t->state_Order = 0;
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
            _ctr_t->state_Order = 1;}
        }
        break;
    }
    case 1:
    {
				up_true_slope=0;
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
            vTaskDelay(1500);
            _ctr_t->state_Order = 3;
        }
        break;
    }

    case 3:
    {
        vTaskDelay(3500);
        normal_time_cnt = 0;
        imu_priority = 0;
        _ctr_t->state_Order = -1;
			//第2或第三圈，并且没转过弯
			if(loop_times>0&&exist_turn_rectangle_flag==0)
			{
			gray_state_Ctr.state_Order = -2;
			}else 
			{
			gray_state_Ctr.state_Order = 0;
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
        if (white_Cnt >= 5 &&
            Gw_GrayscaleSensor_right.data[0] == 1 && Gw_GrayscaleSensor_right.data[1] == 1 && Gw_GrayscaleSensor_right.data[2] == 1 && Gw_GrayscaleSensor_right.data[3] == 1 && Gw_GrayscaleSensor_right.data[4] == 1 && Gw_GrayscaleSensor_right.data[5] == 1 &&
            Gw_GrayscaleSensor_right.data[6] == 1 && Gw_GrayscaleSensor_right.data[7] == 1)
            _ctr_t->state_Order = 1;
        break;
    }

    // 静止
    case 1:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 2;
				exist_turn_rectangle_flag=1;
        break;
    }
    // 状态2旋转
    case 2:
    {

        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 3;
            turn_times++;
        }
        break;
    }
    // 直线慢速
    case 3:
    {
        if (_ctr_t->data[1] == 0 && _ctr_t->data[2] == 0 && _ctr_t->data[3] == 0 && _ctr_t->data[4] == 0 && _ctr_t->data[5] == 0 && _ctr_t->data[6] == 0)
            _ctr_t->state_Order = 4;
        break;
    }
        // 停下等待1s
    case 4:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 5;
        break;
    }
    // 旋转
    case 5:
    {
        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 6;
            turn_times++;
        }
        break;
    }

    case 6:
    {
        int white_Cnt = 0;
        for (int i = 0; i < 7; i++)
        {
            if (_ctr_t->data[i] == 0)
                white_Cnt++;
        }
        if (white_Cnt >= 5 &&
            Gw_GrayscaleSensor_right.data[0] == 1 && Gw_GrayscaleSensor_right.data[1] == 1 && Gw_GrayscaleSensor_right.data[2] == 1 && Gw_GrayscaleSensor_right.data[3] == 1 && Gw_GrayscaleSensor_right.data[4] == 1 && Gw_GrayscaleSensor_right.data[5] == 1 &&
            Gw_GrayscaleSensor_right.data[6] == 1 && Gw_GrayscaleSensor_right.data[7] == 1)
            _ctr_t->state_Order = 7;
        break;
    }

    // 静止
    case 7:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 8;

        break;
    }
    // 状态2旋转
    case 8:
    {

        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 9;
            turn_times++;
        }
        break;
    }
    // 直线慢速
    case 9:
    {
        if (_ctr_t->data[1] == 0 && _ctr_t->data[2] == 0 && _ctr_t->data[3] == 0 && _ctr_t->data[4] == 0 && _ctr_t->data[5] == 0 && _ctr_t->data[6] == 0)
            _ctr_t->state_Order = 10;
        break;
    }
        // 停下等待1s
    case 10:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 11;
        break;
    }

        // 旋转
    case 11:
    {
        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 0;
            turn_times++;
        }
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
        speed_normal = 1.1;
        car->vel_Control();
        break;
    }
    case normal:
    {
        car->update_vel_flag = 1;
        speed_normal = 1.2;
        car->vel_Control();
        break;
    }
    case speed_up:
    {

        car->update_vel_flag = 1;
        speed_normal = 1.35;
        car->vel_Control();
        vTaskDelay(50);
        speed_normal = 1.75;
        car->vel_Control();
        break;
    }
    case slow_down:
    {
        car->update_vel_flag = 1;
        speed_normal = 0.8;
        car->vel_Control();
        break;
    }
    case slow_down_stone:
    {
        car->update_vel_flag = 0;
        float yaw_limit = 0;
        if (yaw_true >= imu_yaw_limit)
        {
            yaw_limit = imu_yaw_limit;
        }
        else if (yaw_true <= -imu_yaw_limit)
        {

            yaw_limit = -imu_yaw_limit;
        }
        else
        {
            yaw_limit = yaw_true;
        }

        pwm_l = 8 + yaw_limit * 0.5;
        pwm_r = 8 - yaw_limit * 0.5;
        break;
    }

    case turn_state:
    {
        car->update_vel_flag = 0;
        pwm_l = -7;
        pwm_r = 13;
        break;
    }

    case turn_state_second:
    {
        car->update_vel_flag = 0;
        pwm_l = -8;
        pwm_r = 12;
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

void car_patter_Switch(car_state *car)
{
        if (turn_times >= 4)
        {
            vTaskDelay(300);
            imu_priority = 1;
            turn_times = 0;
            imu_pitch_state_Ctr.state_Order = 0;
        }

				if(loop_times==3)
				{
				vTaskDelay(1000);
				stop_flag=1;
				}
				
        if (imu_priority == 1)
        {
            if (imu_pitch_state_Ctr.state_Order == 0)
            {
                car->_car_mode = begin_state;
            }
            else if (imu_pitch_state_Ctr.state_Order == 1)
            {
                car->_car_mode = speed_up;
            }
            else if (imu_pitch_state_Ctr.state_Order == 2)
            {
                car->_car_mode = slow_down;
            }
            else if (imu_pitch_state_Ctr.state_Order == 3)
            {
                car->_car_mode = slow_down_stone;
            }
        }
        else if (imu_priority == 0)
        {
						if(loop_times==0)
						{
						  if (normal_time_cnt >= 0 && gray_state_Ctr.state_Order == 0)
            {
                normal_time_cnt++;
            }
            if (normal_time_cnt >= 200)
            {
                gray_state_Ctr.state_Order = -1;
                normal_time_cnt = -1;
            }				
						}
          	if(gray_state_Ctr.state_Order == -2)
						{
								car->_car_mode = slow_down;
								vTaskDelay(2000);
								gray_state_Ctr.state_Order = -1;					
						}
            if (gray_state_Ctr.state_Order == -1)
                car->_car_mode = normal;
            if (gray_state_Ctr.state_Order == 0)
                car->_car_mode = slow_down;
            else if (gray_state_Ctr.state_Order == 1)
            {
							    normal_time_cnt = -1;
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
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 8)
            {
                car->_car_mode = turn_state_second;
            }
            else if (gray_state_Ctr.state_Order == 9)
            {
                car->_car_mode = normal;
            }
            else if (gray_state_Ctr.state_Order == 10)
            {
                car->_car_mode = stop_car;
            }
            else if (gray_state_Ctr.state_Order == 11)
            {
                car->_car_mode = turn_state_second;
            }
        }



}
