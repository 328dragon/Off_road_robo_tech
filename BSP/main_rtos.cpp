#include "main_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "ccd.h"
#include "control.h"
#include "bsp_usart.h"
#include "gw_grasycalse.h"
#include "state_control.h"
#include "H30_analysis_data.h"
#include "SR04.h"
__IO int start_flag = 0;
__IO int stop_flag = 0;
extern float speed_normal;
extern float pwm_l;
extern float pwm_r;
extern int turn_rectngle_flag;

state_ctrl_t imu_yaw_state_Ctr = {0, 0, 0, NULL, H30_yaw_state_ctrl};
state_ctrl_t imu_pitch_state_Ctr = {0, 0, 0, NULL, H30_pitch_state_ctrl};
state_ctrl_t gray_state_Ctr = {0, 0, 0, NULL, gw_state_Ctrl};
state_ctrl_t SR04_state_Ctr = {0, 0, 0, NULL, SR04_state_Controller};


// 感为
GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor;
GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor_right;
// 超声波
SR04_t front_sr04;
// ccd数据
CCD_t front_ccd;
// 陀螺仪
extern float pitch_true;
extern float roll_true;
extern float yaw_true;
// 串口配置
USARTInstance uart2 = {0};
USARTInstance uart3 = {0};
void usart2_callback(void)
{
    // 帧头帧尾校验
    if (uart2.recv_buff[0] == 0xA5 && uart2.recv_buff[4] == 0x5A)
    {
        //"start检验"
        if (uart2.recv_buff[1] == 0x13 && uart2.recv_buff[2] == 0x00 && uart2.recv_buff[3] == 0x13)
        {
//            start_flag = 1;
        }
        if (uart2.recv_buff[1] == 0x0A && uart2.recv_buff[2] == 0x42 && uart2.recv_buff[3] == 0x4C)
        {
            stop_flag = 1;
        }
    }
}

void usart3_callback(void)
{
    H30_analysis_data(uart3.recv_buff);
}

USART_Init_Config_s uart2_cfg = {
    .recv_buff_size = 10,
    .usart_handle = &huart2,
    .module_callback = usart2_callback,
};
USART_Init_Config_s uart3_cfg = {
    .recv_buff_size = 100,
    .usart_handle = &huart3,
    .module_callback = usart3_callback,
};

#define abs(x) (((x) > 0) ? (x) : (-(x)))

using namespace Motor;
Motor::dc_motor motorl(htim8, TIM_CHANNEL_4, PH1_GPIO_Port, PH1_Pin, 1);
Motor::dc_motor motorr(htim8, TIM_CHANNEL_3, PH2_GPIO_Port, PH2_Pin, 0);
car_state car;

TaskHandle_t main_rtos_handle;       // 主函数
TaskHandle_t state_update_handle;    // 状态更新
TaskHandle_t data_processing_handle; // 数据获取及处理
TaskHandle_t pattern_switch_handle;  // 模式切换
TaskHandle_t gray_read_handle;       // 灰度传感器
TaskHandle_t H30_state_handle;       // 陀螺仪结算
void gray_read_task(void *pvParameters);
void state_update(void *pvparameters);
void data_processing(void *pvparameters);
void pattern_switch(void *pvparameters);
void H30_state_task(void *pvparameters);

void main_rtos(void)
{

    USARTRegister(&uart2, &uart2_cfg);
    memset(uart2.recv_buff, 0, uart2.recv_buff_size);
    HAL_Delay(1000);
    USARTRegister(&uart3, &uart3_cfg);
    memset(uart3.recv_buff, 0, uart3.recv_buff_size);

    // 超声波
    HAL_TIM_Base_Start_IT(&htim11);
    //	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
    SR04_Register(&front_sr04, SR04_TRIG_GPIO_Port, SR04_TRIG_Pin, &htim1, TIM_CHANNEL_2);

    // 灰度
    Gw_GrayscaleSensor = GW_grasycalse::Gw_Grayscale_t(&hi2c1, GW_GRAY_ADDR_DEF);
    Gw_GrayscaleSensor_right = GW_grasycalse::Gw_Grayscale_t(&hi2c1, GW_GRAY_ADDR_DEF_right);
    motorl.motor_init();
    motorr.motor_init();
    car.car_init();
    CCD_Init(&front_ccd, DR_IRQ_GPIO_Port, DR_IRQ_Pin, SPI_CS_GPIO_Port, SPI_CS_Pin, &hspi3);

    BaseType_t task1 = xTaskCreate(state_update, "state_update", 200, NULL, 4,
                                   &state_update_handle);
    BaseType_t task2 = xTaskCreate(data_processing, "data_processing", 200, NULL, 4,
                                   &data_processing_handle);
    BaseType_t task3 = xTaskCreate(pattern_switch, "pattern_switch", 300, NULL, 4,
                                   &pattern_switch_handle);
    BaseType_t ok4 = xTaskCreate(H30_state_task, "mpu6050_read_task", 300, NULL, 2, &H30_state_handle);
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
void H30_state_task(void *pvparameters)
{

    while (1)
    {
        H30_pitch_state_ctrl(&imu_pitch_state_Ctr, &pitch_true);
				H30_yaw_state_ctrl(&imu_yaw_state_Ctr,&yaw_true);
			
        SR04_GetData(&front_sr04);
        SR04_state_Controller(&SR04_state_Ctr, &front_sr04.distant);
			
			char gray_order[10];
			sprintf(gray_order,"%d\r\n",gray_state_Ctr.state_Order);
			HAL_UART_Transmit_DMA(&huart2,(uint8_t*)gray_order,10);
        vTaskDelay(100);
    }
}

void gray_read_task(void *pvParameters)
{
    gray_state_Ctr.data = Gw_GrayscaleSensor.data;

    while (1)
    {
        Gw_GrayscaleSensor.read_data_gpio();
       Gw_GrayscaleSensor_right.read_data();
        gray_state_Ctr.state_func(&gray_state_Ctr, NULL);
        vTaskDelay(20);
    }
}
void state_update(void *pvparameters)
{
    while (1)
    {
        car_speed_state_switch(&car);
        vTaskDelay(10);
    }
}

void data_processing(void *pvparameters)
{
    while (1)
    {
        ccd_data_process(&front_ccd);
        car.pos_update(front_ccd.ccd_slope_max_pos);
        // 任务延迟

        vTaskDelay(10);
    }
}
void pattern_switch(void *pvparameters)
{
    while (1)
    {
        car_patter_Switch(&car);
        vTaskDelay(10);
    }
}
