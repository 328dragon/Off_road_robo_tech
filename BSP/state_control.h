#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gw_grasycalse.h"
#include "H30_analysis_data.h"
#include "SR04.h"
#include "control.h"

typedef struct  _
{
__IO int state_Order;
double  cin;
double cout;
uint8_t *data;
 void (*state_func)(struct _ *,  float *cin);

}state_ctrl_t;

void H30_yaw_state_ctrl(state_ctrl_t *_ctr_t, float *cin);
void H30_state_ctrl(state_ctrl_t *_ctr_t, float *cin);
void gw_state_Ctrl(state_ctrl_t *_ctr_t, float *cin);
void SR04_state_Controller(state_ctrl_t *_ctr_t, float *cin);
void car_speed_state_switch(car_state *car);
void car_patter_Switch(car_state *car);
#endif 

