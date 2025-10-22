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

void H30_state_ctrl(state_ctrl_t *_ctr_t, float *cin);
void gw_state_Ctrl(state_ctrl_t *_ctr_t, float *cin);
void SR04_state_Controller(state_ctrl_t *_ctr_t, float *cin);

state_ctrl_t imu_state_Ctr = {0, 0, 0, NULL, H30_state_ctrl};
state_ctrl_t gray_state_Ctr = {0, 0, 0, NULL, gw_state_Ctrl};
state_ctrl_t SR04_state_Ctr = {0, 0, 0, NULL, SR04_state_Controller};

int imu_priority = 1;
int finish_one_loop = 0;
int obstacle_cnt=0;
__IO	int normal_time_cnt=-1;
// 感为
GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor;
GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor_right;
// 超声波
SR04_t front_sr04;
// ccd数据
CCD_t front_ccd;
//********陀螺仪********////////

#define imu_yaw_limit 4
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

int turn_times = 0;

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
            start_flag = 1;
        }
        if (uart2.recv_buff[1] == 0x0A && uart2.recv_buff[2] == 0x42 && uart2.recv_buff[3] == 0x4C)
        {
            stop_flag = 1;
        }
    }
}

void usart3_callback(void)
{
    if (uart3.recv_buff[0] == 0x59 && uart3.recv_buff[1] == 0x53 && uart3.recv_buff[33] == 0x40 && uart3.recv_buff[34] == 0x0C)
    {
        // pitch_all_int=uart3.recv_buff[46]<<24+uart3.recv_buff[45]<<16+uart3.recv_buff[44]<<8+uart3.recv_buff[43];
        pitch_all_int = (int)((uart3.recv_buff[38] << 24) | (uart3.recv_buff[37] << 16) | (uart3.recv_buff[36] << 8) | uart3.recv_buff[35]);
        roll_all_int = (int)((uart3.recv_buff[42] << 24) | (uart3.recv_buff[41] << 16) | (uart3.recv_buff[40] << 8) | uart3.recv_buff[39]);
        yaw_all_int = (int)((uart3.recv_buff[46] << 24) | (uart3.recv_buff[45] << 16) | (uart3.recv_buff[44] << 8) | uart3.recv_buff[43]);
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
    }
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

void SR04_state_Controller(state_ctrl_t *_ctr_t, float *cin)
{
    switch (_ctr_t->state_Order)
    {
    case 0:
    {
        if (*cin < 20.0)
        {
            _ctr_t->state_Order = 1;
        }
        break;
    }
    //前方是第一个障碍
    case 1:
    {
         if (*cin >60)
        {
            obstacle_cnt++;
            vTaskDelay(200);
            _ctr_t->state_Order = 0;
        } 
        break;
    }
    default:
        break;
    }
}

void H30_state_ctrl(state_ctrl_t *_ctr_t, float *cin)
{
    switch (_ctr_t->state_Order)
    {
    case 0:
    {
        if (*cin < -12.0)
        {
            imu_priority = 1;
            _ctr_t->state_Order = 1;
        }
        break;
    }
    case 1:
    {
        if (*cin < 8.0 && *cin > -12.0)
        {
            _ctr_t->state_Order = 2;
        }
        break;
    }
    case 2:
    {
        if (*cin > 8.0)
        {
					vTaskDelay(1500);
            _ctr_t->state_Order = 3;
        }
        break;
    }
		
		case 3:
		{
			vTaskDelay(4000);
			normal_time_cnt=0;
     imu_priority = 0;
            _ctr_t->state_Order = -1;
            gray_state_Ctr.state_Order = 0;
		}

    default:
        break;
    }
}

// 内圈感为状态机
void gw_state_Ctrl(state_ctrl_t *_ctr_t, float *cin)
{
    switch (_ctr_t->state_Order)
    {
        // 状态0加速
    case 0:
    {
        int white_Cnt = 0;
        for (int i = 0; i < 7; i++)
        {
            if (_ctr_t->data[i] == 0)
                white_Cnt++;
        }
        if (white_Cnt >= 5 &&
            Gw_GrayscaleSensor_right.data[0] == 1 && Gw_GrayscaleSensor_right.data[1] == 1 && Gw_GrayscaleSensor_right.data[2] == 1 && Gw_GrayscaleSensor_right.data[3] == 1 && Gw_GrayscaleSensor_right.data[4] == 1 && Gw_GrayscaleSensor_right.data[5] == 1 &&
            Gw_GrayscaleSensor_right.data[6] == 1 && Gw_GrayscaleSensor_right.data[7] == 1)
            _ctr_t->state_Order = 1;
        break;
    }

    // 静止
    case 1:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 2;

        break;
    }
    // 状态2旋转
    case 2:
    {

        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 3;
            turn_times++;
        }
        break;
    }
    // 直线慢速
    case 3:
    {
        if (_ctr_t->data[1] == 0 && _ctr_t->data[2] == 0 && _ctr_t->data[3] == 0 && _ctr_t->data[4] == 0 && _ctr_t->data[5] == 0 && _ctr_t->data[6] == 0)
            _ctr_t->state_Order = 4;
        break;
    }
        // 停下等待1s
    case 4:
    {
        vTaskDelay(300);
        _ctr_t->state_Order = 5;
        break;
    }
    // 旋转
    case 5:
    {
        if ((Gw_GrayscaleSensor.data[3] == 1 && Gw_GrayscaleSensor.data[4] == 1 && Gw_GrayscaleSensor.data[5] == 1) && (Gw_GrayscaleSensor.data[6] == 0 && Gw_GrayscaleSensor.data[7] == 0))
        {
            _ctr_t->state_Order = 0;
            turn_times++;
        }
        break;
    }

    default:
        break;
    }
}

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
    BaseType_t task3 = xTaskCreate(pattern_switch, "pattern_switch", 200, NULL, 4,
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
        //			char tx_h30[12];
        //			sprintf(tx_h30,":%f\r\n",pitch_true);
        // HAL_UART_Transmit(&huart3,(uint8_t*)tx_h30,12,20);
        H30_state_ctrl(&imu_state_Ctr, &pitch_true);
        SR04_GetData(&front_sr04);
        SR04_state_Controller(&SR04_state_Ctr, &front_sr04.distant);
        vTaskDelay(100);
    }
}

void gray_read_task(void *pvParameters)
{
    gray_state_Ctr.data = Gw_GrayscaleSensor.data;

    while (Gw_GrayscaleSensor.gw_ping() || Gw_GrayscaleSensor_right.gw_ping())
    {
        vTaskDelay(100);
    }
    while (1)
    {
        Gw_GrayscaleSensor.read_data();
        Gw_GrayscaleSensor_right.read_data();
        gray_state_Ctr.state_func(&gray_state_Ctr, NULL);
        vTaskDelay(20);
    }
}
void state_update(void *pvparameters)
{
    while (1)
    {
        switch (car._car_mode)
        {
        case normal:
        {
            car.update_vel_flag = 1;
            speed_normal = 1.3;
            car.vel_Control();
            break;
        }
        case speed_up:
        {

            car.update_vel_flag = 1;
						speed_normal = 1.45;
						car.vel_Control();
						vTaskDelay(30);
            speed_normal = 1.8;
            car.vel_Control();
            break;
        }
        case slow_down:
        {
            car.update_vel_flag = 1;
            speed_normal = 0.8;
            car.vel_Control();
            break;
        }
        case slow_down_stone:
        {
            car.update_vel_flag = 0;
					float yaw_limit=0;
					if(yaw_true>=imu_yaw_limit)
					{
					yaw_limit=imu_yaw_limit;
					}else if(yaw_true<=-imu_yaw_limit)
					{
					
					yaw_limit=-imu_yaw_limit;}
					else 
					{
					yaw_limit=yaw_true;
					}
					
            pwm_l = 8+yaw_limit*0.5;
            pwm_r = 8-yaw_limit*0.5;
            break;
        }

        case turn_state:
        {
            car.update_vel_flag = 0;
            pwm_l = -7;
            pwm_r = 13;
            break;
        }
        case turn_state_second:
        {
            car.update_vel_flag = 0;
            pwm_l = -8;
            pwm_r = 14;
            break;
        }
        case stop_car:
        {
            car.update_vel_flag = 0;
            pwm_l = 0;
            pwm_r = 0;
            break;
        }

        default:
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
        // 任务延迟

        vTaskDelay(10);
    }
}
void pattern_switch(void *pvparameters)
{
	


    while (1)
    {

        if (finish_one_loop == 0)
        {

            if (turn_times >= 4)
            {
							  vTaskDelay(300);
                imu_priority = 1;
                turn_times = 0;
                imu_state_Ctr.state_Order = 0;
            }

            if (imu_priority == 1)
            {
                if (imu_state_Ctr.state_Order == 0)
                {
                    car._car_mode = slow_down;
                }
                else if (imu_state_Ctr.state_Order == 1)
                {
                    car._car_mode = speed_up;
                }
                else if (imu_state_Ctr.state_Order == 2)
                {
                    car._car_mode = slow_down;
                }
								else if (imu_state_Ctr.state_Order == 3)
                {
                    car._car_mode = slow_down_stone;
                }
            }
            else if (imu_priority == 0)
            {
							if(normal_time_cnt>=0&&gray_state_Ctr.state_Order == 0)
							{
							normal_time_cnt++;
							}
							if(normal_time_cnt>=200)
							{							
											gray_state_Ctr.state_Order = -1;
								normal_time_cnt=-1;
							}
								if (gray_state_Ctr.state_Order == -1)
                    car._car_mode = normal;
                if (gray_state_Ctr.state_Order == 0)
                    car._car_mode = slow_down;
                else if (gray_state_Ctr.state_Order == 1)
                {
                    car._car_mode = stop_car;
                }
                else if (gray_state_Ctr.state_Order == 2)
                {
                    car._car_mode = turn_state;
                }
                else if (gray_state_Ctr.state_Order == 3)
                {
                    car._car_mode = slow_down;
                }
                else if (gray_state_Ctr.state_Order == 4)
                {
                    car._car_mode = stop_car;
                }
                else if (gray_state_Ctr.state_Order == 5)
                {
                    car._car_mode = turn_state;
                }
            }
        }

        // 最后停下逻辑
        if (finish_one_loop == 1)
        {
            car._car_mode = stop_car;
        }

        vTaskDelay(10);
    }
}
