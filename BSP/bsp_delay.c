#include "bsp_delay.h"

int delay_no_conflict(int *delay_temp_count, int delay_time_base_multiple)
{
  (*delay_temp_count)++;
  if (*delay_temp_count >= delay_time_base_multiple)
  {
    *delay_temp_count = 0;
    return 1;
  }
  else
    return 0;
}