#include "main_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "ccd.h"
#include "control.h"
#include "bsp_usart.h"
#include "mpu6050.h"
#include "kalman.h"
 #include "gw_grasycalse.h"
__IO int start_flag=0;		
__IO int stop_flag=0;
extern float  speed_normal;
extern float pwm_l;
extern float pwm_r;
USARTInstance uart2 = {0};
void usart2_callback(void)
{
	//帧头帧尾校验
if(uart2.recv_buff[0]==0xA5&&uart2.recv_buff[4]==0x5A)
{
	//"start检验"
if(uart2.recv_buff[1]==0x13&&uart2.recv_buff[2]==0x00&&uart2.recv_buff[3]==0x13)
{
start_flag=1;
}
if(uart2.recv_buff[1]==0x0A&&uart2.recv_buff[2]==0x42&&uart2.recv_buff[3]==0x4C)
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
Motor::dc_motor motorl(htim8, TIM_CHANNEL_4, PH1_GPIO_Port, PH1_Pin,  1);
Motor::dc_motor motorr(htim8, TIM_CHANNEL_3, PH2_GPIO_Port, PH2_Pin,  0);
car_state car;
GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor;
extern MPU6050_Handle_t mpu6050_dragon;
TaskHandle_t main_rtos_handle;       // 主函数
TaskHandle_t state_update_handle;    // 状态更新
TaskHandle_t data_processing_handle; // 数据获取及处理
TaskHandle_t pattern_switch_handle;  // 模式切换
TaskHandle_t gray_read_handle;        // 灰度传感器
TaskHandle_t	mpu6050_read_handle;//陀螺仪结算
void gray_read_task(void *pvParameters);
void state_update(void *pvparameters);
void data_processing(void *pvparameters);
void pattern_switch(void *pvparameters);
void mpu6050_read_task(void *pvparameters);


void main_rtos(void)
{
    USARTRegister(&uart2, &uart2_cfg);
	memset(uart2.recv_buff, 0, uart2.recv_buff_size);
    //mpu6050

		MPU6050_Handle_Init(&hi2c2,0xD0,&mpu6050_dragon);
//灰度
	 Gw_GrayscaleSensor = GW_grasycalse::Gw_Grayscale_t(&hi2c1, GW_GRAY_ADDR_DEF);
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
		 BaseType_t ok4 = xTaskCreate(mpu6050_read_task, "mpu6050_read_task", 300, NULL, 2, &mpu6050_read_handle);
	 BaseType_t ok5 = xTaskCreate(gray_read_task, "gray_read_task", 300, NULL, 2, &gray_read_handle);
	
    if (task1 != pdPASS || task2 != pdPASS || task3 != pdPASS)
    {

        // 任务创建失败，进入死循环
        while (1)
        {
            // printf("create task failed\n");
        }
    }
}
void mpu6050_read_task(void *pvparameters)
{

while(1)
{

	MPU6050_Handle_GET_Data(&mpu6050_dragon);
 Kalman_MPU6050_Filter(&mpu6050_dragon);
	
    vTaskDelay(100);
}
}

void gray_read_task(void *pvParameters)
{
  while (Gw_GrayscaleSensor.gw_ping())
  {
    vTaskDelay(100);
  }
  while (1)
  {
    Gw_GrayscaleSensor.read_data();
    vTaskDelay(20);
  }
}
void state_update(void *pvparameters)
{
    while (1)
    {
			switch(car._car_mode)
			{
				case normal:
				{
//					speed_normal=1.85;
						car.update_vel_flag=1;
						speed_normal=1.3;
		
						car.vel_Control();
						break;
				}			
				case slow_down:
				{
					car.update_vel_flag=1;
					speed_normal=0.9;
						car.vel_Control();
				break;
				}
				case turn_state:
				{
					car.update_vel_flag=0;
						pwm_l=-8;
					pwm_r=12;
					break;
				}				
				case stop_car:
				{
					car.update_vel_flag=0;
						pwm_l=0;
					pwm_r=0;
				break;
				}
					
				default :
					break;
			}
       
        vTaskDelay(10);
    }
}

void data_processing(void *pvparameters)
{
    while (1)
    {
        ccd_data_process(&front_ccd);
						car.pos_update(front_ccd.ccd_slope_max_pos);
//        // 发送给电脑中心位置，调试用

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
