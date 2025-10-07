#include "control.h"
#define left_ccd_limit 78
#define right_ccd_limit 22
#define ccd_center 50

pid_t revise_ccd_pid;
extern Motor::dc_motor motorl;
extern Motor::dc_motor motorr;

float  speed_normal=1;
float  speed_revise =0;

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
PID_struct_init(&revise_ccd_pid,POSITION_PID,0.1,0.02,0,0,0);

}

car_state::car_state()
{   
    target_speedl = speed_normal;
    target_speedr = speed_normal; 
    car_pos = straight;
    car_line_error = 0;
    data_processing_flag = 0;
}

void car_state::vel_Control(void)
{   
    if(data_processing_flag == 0)
    {
			speed_revise=revise_proportional(speed_normal);
        target_speedl = speed_normal + car_pos*car_line_error*speed_revise;
        target_speedr = speed_normal - car_pos*car_line_error*speed_revise;

        motorl.motor_close_vel(target_speedl);
        motorr.motor_close_vel(target_speedr);
        //此处需要速度大小到pwm的转换算法
        data_processing_flag = 1;
    }
}
void car_state::pos_update(int ccd_pos)
{
    if(ccd_pos<=(ccd_center+2)&&ccd_pos>=(ccd_center-2))
    {
        car_pos = straight;
        car_line_error = 0;
    }
    else if (ccd_pos<(ccd_center+2)&&ccd_pos>=(right_ccd_limit))
    {
        car_pos=right;
        car_line_error=ccd_center-ccd_pos;
    }
    else if(ccd_pos>(ccd_center-2)&&ccd_pos<=(left_ccd_limit))
    {
        car_pos=left;
        car_line_error=ccd_pos-ccd_center;
    }
data_processing_flag=0;
}