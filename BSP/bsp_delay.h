#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H
#ifdef __cplusplus
extern "C"
{
#endif
    // 开始
int delay_no_conflict(int *delay_temp_count, int delay_time_base_multiple);

#ifdef __cplusplus
}
#endif

#endif // !__BSP_DELAY_H

