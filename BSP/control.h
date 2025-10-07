#ifndef __CONTROL_H
#define __CONTROL_H

#include "motor.h"
#include "pid.h"
#include "ccd.h"
typedef enum
{
    straight=0,
    left=1,
    right=-1
} car_position_t;




class car_state
{
    public:

        float    target_speedl;
        float   target_speedr;
        car_position_t   car_pos;
        int   car_line_error;
        int   data_processing_flag;


        void vel_Control(void);
        void pos_update(int ccd_pos);
        car_state();
       
};

extern car_state car;
#endif