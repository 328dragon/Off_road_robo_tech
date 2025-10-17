#include "main_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "ccd.h"
#include "control.h"
#include "bsp_usart.h"
 #include "SR04.h"
__IO int start_flag=0;		
__IO int stop_flag=0;
extern float  speed_normal ;
extern  int turn_rectngle_flag;
int turn_rectangle_cnt=0;
int turn_begin_flg=0;
extern int exsit_turn_rectangle;
extern float pwm_l;
extern float pwm_r;
USARTInstance uart3 = {0};
void usart3_callback(void)
{
	//帧头帧尾校验
if(uart3.recv_buff[0]==0xA5&&uart3.recv_buff[4]==0x5A)
{
	//"start检验"
if(uart3.recv_buff[1]==0x13&&uart3.recv_buff[2]==0x00&&uart3.recv_buff[3]==0x13)
{
start_flag=1;
}
if(uart3.recv_buff[1]==0x0A&&uart3.recv_buff[2]==0x42&&uart3.recv_buff[3]==0x4C)
{
stop_flag=1;
}

}
	
}

USART_Init_Config_s uart3_cfg = {
    .recv_buff_size = 10,
    .usart_handle = &huart3,
    .module_callback = usart3_callback,
};

SR04_t SR04_front;

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
    USARTRegister(&uart3, &uart3_cfg);
	memset(uart3.recv_buff, 0, uart3.recv_buff_size);
    //超声波
	HAL_TIM_Base_Start_IT(&htim11);												 
  SR04_Register(&SR04_front, SR04_TIRG_GPIO_Port, SR04_TIRG_Pin, &htim5, TIM_CHANNEL_3);
    motorl.motor_init();
    motorr.motor_init();
		car.car_init();
    CCD_Init(&front_ccd, DR_IRQ_GPIO_Port, DR_IRQ_Pin, SPI_CS_GPIO_Port, SPI_CS_Pin, &hspi3);
//	pid_reset(&motorr.vel_pid,10,0.009,0.13);
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
//					pwm_l=-10;
//					pwm_r=10;
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
	int slow_turn_flag=0;
    while (1)
    {
//			if(stop_flag==0)
//			{
//			      char tx_ccd[27];
//        sprintf(tx_ccd, ":%d,%d,%d,%f,%f\r\n", front_ccd.slope_up_max_pos,front_ccd.ccd_slope_max_pos,front_ccd.slope_down_max_pos,car.target_speedl,car.target_speedr);
//        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_ccd, 27);
//			}
			if(exsit_turn_rectangle==1)
			{
			if(turn_rectngle_flag==1)
			{
			car._car_mode=stop_car;				
				turn_rectangle_cnt++;
				slow_turn_flag=0;
			vTaskDelay(50);
//			car._car_mode=turn_state;
			turn_rectngle_flag=0;
			}
			
			if(car.car_pos !=straight&&slow_turn_flag==0)
			{
			car._car_mode=turn_state;		
			}
			
			
//		if(car._car_mode==turn_state&&(front_ccd.ccd_slope_max_pos>=90&&front_ccd.ccd_slope_max_pos<=110)&&(turn_rectangle_cnt==1||turn_rectangle_cnt==3))
//		{
//			car._car_mode=stop_car;
//		}
//		else if(car._car_mode==turn_state&&(front_ccd.ccd_slope_max_pos>=90&&front_ccd.ccd_slope_max_pos<=110)&&(turn_rectangle_cnt==2||turn_rectangle_cnt==4))
//		{
//			car._car_mode=stop_car;
//		}
//			
			
			 if((front_ccd.ccd_slope_max_pos>=90&&front_ccd.ccd_slope_max_pos<=100)&&(turn_rectangle_cnt==1||turn_rectangle_cnt==3))
			{
				slow_turn_flag=1;
			car._car_mode=slow_down;
			}else if((front_ccd.ccd_slope_max_pos>=90&&front_ccd.ccd_slope_max_pos<=100)&&(turn_rectangle_cnt==0||turn_rectangle_cnt==2)||turn_rectangle_cnt==4)
			{
					slow_turn_flag=1;
			car._car_mode=normal;			
			}
			
			}

			
			
			
             SR04_GetData(&SR04_front);
//			      char vel_pid[21];
//        sprintf(vel_pid, ":%f,%f\r\n",motorl.current_wheel_speed,motorr.current_wheel_speed);
//        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)vel_pid, 21);		  
        vTaskDelay(100);
    }
}
