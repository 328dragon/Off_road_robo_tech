#include "main_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "ATK_MS53L0M.h"
extern uint8_t dma_rx_buf[1024];
extern __IO int ccd_data_ok;
uint8_t ccd_data[101];
using namespace Motor;
extern Motor::dc_motor motorl;
extern Motor::dc_motor motorr;

TaskHandle_t main_rtos_handle;       // 主函数
TaskHandle_t state_update_handle;    // 状态更新
TaskHandle_t data_processing_handle; // 数据获取及处理
TaskHandle_t pattern_switch_handle;  // 模式切换

void state_update(void *pvparameters);
void data_processing(void *pvparameters);
void pattern_switch(void *pvparameters);

void main_rtos(void)
{

    // motorl.motor_init();
    // motorr.motor_init();

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
        vTaskDelay(10);
    }
}
void data_processing(void *pvparameters)
{
    while (1)
    {
        if (ccd_data_ok == 1)
        {
            for (int i = 0; i < 101; i++)
            {
                for (int j = 0; j < 10; j++)
                {
                    ccd_data[i] += dma_rx_buf[i * 10 + j] / 10;
                }
            }
        }
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
