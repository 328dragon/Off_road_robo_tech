#include "main_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "ATK_MS53L0M.h"
#define abs(x) (((x)>0)?(x):(-(x)))
//ccd数据
extern uint8_t ccd_origin_data[1000];
extern __IO int ccd_data_ok;
uint16_t ccd_data[101];
uint16_t ccd_slope_max=0;
int ccd_slope_max_pos=-1;

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

     motorl.motor_init();
    motorr.motor_init();

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
			//压缩数据，10个平均一下
        if (ccd_data_ok == 1)
        {
							memset(ccd_data,0,sizeof(ccd_data));
            for (int i = 0; i < 101; i++)
            {
                for (int j = 0; j < 10; j++)
                {
                    ccd_data[i] += ccd_origin_data[i * 10 + j] / 10;
                }
            }
						ccd_data_ok=0;			
        }
				//遍历所有区域找斜率最大点
				for(int i=0;i<100;i++)
				{

					if(i>22&&i<75)
					{
					int slope_temp=0;
						if(ccd_data[i+1]>=ccd_data[i])
						{
												slope_temp=ccd_data[i+1]-ccd_data[i];
						}
						else{
												slope_temp=ccd_data[i]-ccd_data[i+1];

						}
						if(slope_temp>ccd_slope_max)
						{
						ccd_slope_max=slope_temp;
							ccd_slope_max_pos=i;
						}
					}
				
				}
				ccd_slope_max=0;
				//发送给电脑中心位置，调试用
				char tx_ccd[5];
				sprintf(tx_ccd,":%d\n",ccd_slope_max_pos);
					HAL_UART_Transmit_DMA(&huart3,(uint8_t*)tx_ccd,5);
				//任务延迟
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
