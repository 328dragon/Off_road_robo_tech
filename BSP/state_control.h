#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H

#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct  _
{
int state_Order;
double  cin;
double cout;
uint8_t *data;
 void (*state_func)(struct _ *,  double *cin);

}state_ctrl_t;


#ifdef __cplusplus
}
#endif


#endif 

