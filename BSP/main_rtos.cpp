#include "main_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "ATK_MS53L0M.h"
#include "ccd.h"
#include "control.h"
#include "bsp_usart.h"

__IO int start_flag=0;
__IO int stop_flag=0;

USARTInstance uart2 = {0};
void usart2_callback(void)
{
	//帧头帧尾校验
if(uart2.recv_buff[0]==0xA5&&uart2.recv_buff[4]==0x5A)
{
	//"start检验"
if(uart2.recv_buff[1]==0x00&&uart2.recv_buff[2]==0x13&&uart2.recv_buff[3]==0x13)
{
start_flag=1;
}
if(uart2.recv_buff[1]==0x42&&uart2.recv_buff[2]==0x0A&&uart2.recv_buff[3]==0x4C)
{
stop_flag=1;
}

}
	
}

USART_Init_Config_s uart2_cfg = {
    .recv_buff_size = 10,
    .usart_handle = &huart2,
    .module_callback = usart2_callback,
};

#define abs(x) (((x) > 0) ? (x) : (-(x)))
// ccd数据
CCD_t front_ccd;
using namespace Motor;
Motor::dc_motor motorl(htim8, TIM_CHANNEL_4, PH1_GPIO_Port, PH1_Pin, htim3, 1);
Motor::dc_motor motorr(htim8, TIM_CHANNEL_3, PH2_GPIO_Port, PH2_Pin, htim2, 0);
car_state car;
TaskHandle_t main_rtos_handle;       // 主函数
TaskHandle_t state_update_handle;    // 状态更新
TaskHandle_t data_processing_handle; // 数据获取及处理
TaskHandle_t pattern_switch_handle;  // 模式切换

void state_update(void *pvparameters);
void data_processing(void *pvparameters);
void pattern_switch(void *pvparameters);

void main_rtos(void)
{
  USARTRegister(&uart2, &uart2_cfg);
	  memset(uart2.recv_buff, 0, uart2.recv_buff_size);
    motorl.motor_init();
    motorr.motor_init();
		car.car_init();
    CCD_Init(&front_ccd, DR_IRQ_GPIO_Port, DR_IRQ_Pin, SPI_CS_GPIO_Port, SPI_CS_Pin, &hspi3);
    BaseType_t task1 = xTaskCreate(state_update, "state_update", 200, NULL, 4,
                                   &state_update_handle);
    BaseType_t task2 = xTaskCreate(data_processing, "data_processing", 200, NULL, 4,
                                   &data_processing_handle);
    BaseType_t task3 = xTaskCreate(pattern_switch, "pattern_switch", 200, NULL, 4,
                                   &pattern_switch_handle);
    if (task1 != pdPASS || task2 != pdPASS || task3 != pdPASS)
    {

        // 任务创建失败，进入死循环
        while (1)
        {
            // printf("create task failed\n");
        }
    }
}
void state_update(void *pvparameters)
{
    while (1)
    {
        car.vel_Control();
        vTaskDelay(10);
    }
}
void data_processing(void *pvparameters)
{
    while (1)
    {
        ccd_data_process(&front_ccd);
        car.pos_update(front_ccd.ccd_slope_max_pos);
        // 发送给电脑中心位置，调试用
        char tx_ccd[5];
        sprintf(tx_ccd, ":%d\n", front_ccd.ccd_slope_max_pos);
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_ccd, 5);
        // 任务延迟
        vTaskDelay(10);
    }
}
void pattern_switch(void *pvparameters)
{
    while (1)
    {
        vTaskDelay(10);
    }
}
