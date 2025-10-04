#include "ATK_MS53L0M.h"

tof_data::tof_data()        
{      
    id       = 0;
    rx_ok    = 0;
    rx_len   = 0;
}

void tof_data :: data_processing(void)
{
    char *p=0;   
    uint16_t data=0;
    uint8_t i=0;
    
    while(1)
    {
        if(atk_ms53l0m.rx_ok == 1)
        {
            p = strstr((char *)MS53L0M_RX_buffer, "d:");
            while(*p !='m')
            {
                 if(*p >= '0' && *p <='9')
                     data = data * 10 + (*p - '0');
                 p++;
            }
            
            // data=0;
            rx_ok  = 0;
            rx_len = 0;
        } 
    }
}

void tof_data :: data_receive(void)
{   
    uint8_t res;
    HAL_UART_Receive(&huart3, &res, 1, 1000);

    if (!atk_ms53l0m.rx_ok) 
    {
        atk_ms53l0m.MS53L0M_RX_buffer[atk_ms53l0m.rx_len] = res;
        atk_ms53l0m.rx_len++;

        if (atk_ms53l0m.rx_len > (MS53L0M_MAX_RECEIVE_LEN - 1)) 
            atk_ms53l0m.rx_len = 0; 
    }
}
