#include "control.h"
#define left_ccd_limit 78
#define right_ccd_limit 22

#define ccd_center 50


extern Motor::dc_motor motorl;
extern Motor::dc_motor motorr;
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