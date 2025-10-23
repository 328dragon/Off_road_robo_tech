#include "H30_analysis_data.h"

//********陀螺仪********////////


__IO int first_read = 0;

int32_t pitch_all_int = 0;
static float h30_pitch = 0;
float pitch_zero = 0;
float pitch_true = 0;

int32_t roll_all_int = 0;
static float h30_roll = 0;
float roll_zero = 0;
float roll_true = 0;

int32_t yaw_all_int = 0;
static float h30_yaw = 0;
float yaw_zero = 0;
float yaw_true = 0;


void H30_analysis_data(uint8_t *recv_buff)
{

    if (recv_buff[0] == 0x59 && recv_buff[1] == 0x53 &&recv_buff[33] == 0x40 && recv_buff[34] == 0x0C)
    {
        pitch_all_int = (int)((recv_buff[38] << 24) | (recv_buff[37] << 16) | (recv_buff[36] << 8) | recv_buff[35]);
        roll_all_int = (int)((recv_buff[42] << 24) | (recv_buff[41] << 16) | (recv_buff[40] << 8) | recv_buff[39]);
        yaw_all_int = (int)((recv_buff[46] << 24) | (recv_buff[45] << 16) | (recv_buff[44] << 8) | recv_buff[43]);
        h30_pitch = (pitch_all_int * 0.000001f);
        h30_roll = (roll_all_int * 0.000001f);
        h30_yaw = (yaw_all_int * 0.000001f);
        if (first_read < 20)
        {
            pitch_zero = h30_pitch;
            roll_zero = h30_roll;
            yaw_zero = h30_yaw;
            first_read++;
        }
        else
        {
            pitch_true = h30_pitch - pitch_zero;
            roll_true = h30_roll - roll_zero;
            yaw_true = h30_yaw - yaw_zero;
        }
    }}