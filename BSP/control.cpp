#include "control.h"
#define left_ccd_limit 185
#define right_ccd_limit 15

#define linear_Limit 3
#define far_linear_limit 30
#define Quadratic_limit 35

#define linear_k 0.4
#define far_linear_k 4
#define Quadratic_k 2.5


pid_t revise_ccd_pid;
extern Motor::dc_motor motorl;
extern Motor::dc_motor motorr;

float  speed_normal = 0;
float  speed_revise = 0;

float f(float error, int state)
{   
    if(state == close_linear)
        return linear_k*(error-linear_Limit);
		else if(state == far_linear)
				return far_linear_k*(error-far_linear_limit)+linear_k*(error-linear_Limit);
    else if(state == far)
        return Quadratic_k*(error-Quadratic_limit)*(error-Quadratic_limit) +
					far_linear_k*(error-far_linear_limit)+linear_k*(error-linear_Limit);
//		return far_linear_k*(error-Quadratic_limit)+linear_k*(Quadratic_limit-linear_Limit);
		else
				return 0;
}

float revise_proportional(float _normal_speed)
{
	float temp_revise=0;
if(_normal_speed>0&&_normal_speed<0.5)
{
temp_revise=_normal_speed/60.0f;
}
if(_normal_speed>=0.5&&_normal_speed<=1)
{
temp_revise=_normal_speed/30.0f;
}
if(_normal_speed>1&&_normal_speed<=1.5)
{
temp_revise=_normal_speed/60.0f;//
}

return temp_revise;
}

float revise_pro_pid_cal(pid_t *pid,float get,float set)
{
return pid_calc(pid,get,set);
}

void car_state::car_init(void)
{
PID_struct_init(&revise_ccd_pid,POSITION_PID,1.0,0,0.03,0,0.002);
//	PID_struct_init(&revise_ccd_pid,POSITION_PID,0.8,0,0.025,0,0.002);
}

car_state::car_state()
{   
    target_speedl = speed_normal;
    target_speedr = speed_normal; 
    car_pos = straight;
    car_line_error = 0;
    data_processing_flag = 0;
	_car_mode=normal;
	update_vel_flag=1;
}

void car_state::vel_Control(void)
{   
    if(data_processing_flag == 0)
    {
				speed_revise = revise_pro_pid_cal(&revise_ccd_pid, car_line_error, 0);

        target_speedl = (speed_normal - car_pos*speed_revise);
        target_speedr = speed_normal + car_pos*speed_revise;

        data_processing_flag = 1;
    }
}

void car_state::vel_rectangle_Control(void)
{   
    if(data_processing_flag == 0)
    {
				speed_revise = 0.5;

        target_speedl = (speed_normal - speed_revise);
        target_speedr = speed_normal + speed_revise;

        data_processing_flag = 1;
    }
}


void car_state::pos_update(int ccd_pos)
{
    if(ccd_pos<=(ccd_center+linear_Limit)&&ccd_pos>=(ccd_center-linear_Limit))
    {
        car_pos = straight;
        car_line_error = 0;
    }
    else if (ccd_pos<(ccd_center-linear_Limit)&&ccd_pos>=(right_ccd_limit))
    {
        car_pos=right;
        if(ccd_pos<(ccd_center-Quadratic_limit)&&ccd_pos>=(right_ccd_limit))
        {
            car_error_state = far;
        }
        else if(ccd_pos<(ccd_center-far_linear_limit)&&ccd_pos>=(ccd_center-Quadratic_limit))
        {
            car_error_state = far_linear;
        }
				else if(ccd_pos<(ccd_center-linear_Limit)&&ccd_pos>=(ccd_center-far_linear_limit))
        {
            car_error_state = close_linear;
        }
        car_line_error = f((ccd_center-ccd_pos), car_error_state);
    }
    else if(ccd_pos>(ccd_center+linear_Limit)&&ccd_pos<=(left_ccd_limit))
    {
        car_pos=left;
        if(ccd_pos<(left_ccd_limit)&&ccd_pos>=(ccd_center+Quadratic_limit))
        {
            car_error_state = far;
        }
				else if(ccd_pos<(ccd_center+Quadratic_limit)&&ccd_pos>=(ccd_center+far_linear_limit))
        {
            car_error_state = far_linear;
        }
        else if(ccd_pos<(ccd_center+far_linear_limit)&&ccd_pos>=(ccd_center+linear_Limit))
        {
            car_error_state = close_linear;
        }
        car_line_error = f((ccd_pos-ccd_center), car_error_state);
    }
data_processing_flag=0;
}