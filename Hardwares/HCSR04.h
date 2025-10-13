#ifndef __HCSR04_H
#define	__HCSR04_H

#include "main.h"


extern uint16_t Time;

void HCSR04_Start(void);
uint16_t HCSR04_GetValue(void);
float HCSR04_Length(void);
#endif