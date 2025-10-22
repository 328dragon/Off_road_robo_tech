#ifndef __CONTROL_H
#define __CONTROL_H

#include "motor.h"
#include "pid.h"
#include "ccd.h"
#include "math.h"
typedef enum
{
    straight=0,
    left=1,
    right=-1,
    close_linear=2,
		far_linear=3,
    far=4
} car_position_t;
typedef enum
{
normal=0,
slow_down=1,
turn_state=2,
speed_up=3,
stop_car=4,
turn_state_second=5,
slow_down_stone=6,
speed_up_obstacal=7
	
}car_run_mode;

typedef enum
{
inside=0,
outside=1,
}car_round_state;

class car_state
{
    public:

        float    target_speedl;
        float    target_speedr;
        car_position_t   car_pos;
				car_run_mode  _car_mode;
        int   car_line_error;
        int   car_error_state;
        int   data_processing_flag;
				int update_vel_flag=0;
				void car_init(void);
        void vel_Control(void);
		void vel_rectangle_Control(void);
        void pos_update(int ccd_pos);
        car_state();
       
};

extern car_state car;
#endif