#ifndef H30_ANALYSIS_DATA_H
#define H30_ANALYSIS_DATA_H
#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif
	
	#define imu_yaw_limit 4
	
void H30_analysis_data(uint8_t *recv_buff);

#ifdef __cplusplus
}
#endif

#endif
