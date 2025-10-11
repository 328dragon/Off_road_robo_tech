#ifndef __ATK_MS53L0M_H
#define __ATK_MS53L0M_H

#include "main.h"
#include <string.h>
#include "usart.h"


#define MS53L0M_RX   USART3_RX
#define MS53L0M_TX   USART3_TX

#define MS53L0M_MAX_RECEIVE_LEN     8
#define MS53L0M_MAX_SEND_LEN        8

class tof_data
{
    private:
        uint16_t id;
        bool     rx_ok;
        int      rx_len;
		
		friend void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
		
    public:
        uint8_t MS53L0M_RX_buffer[MS53L0M_MAX_RECEIVE_LEN] = {0};
        uint8_t MS53L0M_TX_buffer[MS53L0M_MAX_SEND_LEN] = {0};			//?????
        void data_receive(void);
        void data_processing(void);
        tof_data();//
};

inline tof_data atk_ms53l0m;  //

#endif