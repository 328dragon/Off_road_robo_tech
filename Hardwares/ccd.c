#include "ccd.h"
#define left_limit 22
#define right_limit 78

int dg_abs(int x)
{
    if (x >= 0)
        return x;
    else
        return -x;
}
void ccd_dr_irq(CCD_t *ccd);
void CCD_Init(CCD_t *ccd,
              GPIO_TypeDef *DR_IRQ_Port, uint16_t _DR_IRQ_Pin, GPIO_TypeDef *CCD_SPI_CS_Port,
              uint16_t CCD_SPI_CS_Pin,
              SPI_HandleTypeDef *ccd_hspi)
{
    ccd->CCD_DR_IRQ_Port = DR_IRQ_Port;
    ccd->CCD_DR_IRQ_Pin = _DR_IRQ_Pin;
    ccd->CCD_SPI_CS_Port = CCD_SPI_CS_Port;
    ccd->CCD_SPI_CS_Pin = CCD_SPI_CS_Pin;
    ccd->ccd_hspi = ccd_hspi;
    ccd->ccd_state = CCD_WAIT;
    ccd->slope_up_max = 0;
    ccd->slope_down_max = 0;
    ccd->slope_up_max_pos = -1;
    ccd->slope_down_max_pos = -1;

    ccd->ccd_slope_max_pos = -1;

    if (!HAL_GPIO_ReadPin(ccd->CCD_DR_IRQ_Port, ccd->CCD_DR_IRQ_Pin))
        ccd_dr_irq(ccd);
}
void ccd_dr_irq(CCD_t *ccd)
{
    if (HAL_GPIO_ReadPin(ccd->CCD_DR_IRQ_Port, ccd->CCD_DR_IRQ_Pin) == GPIO_PIN_SET)
    {
        return;
    }
    else
    {
        HAL_GPIO_WritePin(ccd->CCD_SPI_CS_Port, ccd->CCD_SPI_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Receive_DMA(ccd->ccd_hspi, ccd->ccd_dma_rx_buf, sizeof(ccd->ccd_dma_rx_buf));
    }
}
void ccd_dma_ok(CCD_t *ccd)
{

    HAL_GPIO_WritePin(ccd->CCD_SPI_CS_Port, ccd->CCD_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_SPI_DMAStop(ccd->ccd_hspi);

    for (int i = 0; i < 5; i++) // spi速率过高前面会出现部分上一次数据, 在前几个数据里面查找帧头
    {
        if (*(uint16_t *)(ccd->ccd_dma_rx_buf + i) == 0x2107 &&
            *(uint16_t *)(ccd->ccd_dma_rx_buf + i + 8) == 0x0721)
        {
            memcpy(ccd->ccd_origin_data, ccd->ccd_dma_rx_buf + i + 10, 1000);
            break;
        }
    }
}
// 外部调用
void ccd_exti_callback(CCD_t *ccd, uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ccd->CCD_DR_IRQ_Pin)
    {
        ccd->ccd_state = CCD_WAIT;
        ccd_dr_irq(ccd);
    }
}

void ccd_spi_rx_cplt_callback(CCD_t *ccd, SPI_HandleTypeDef *hspi)
{
    if (hspi == ccd->ccd_hspi)
    {
        ccd->ccd_state = CCD_OK;
        ccd_dma_ok(ccd);
    }
}
/// 处理函数
void ccd_data_process(CCD_t *ccd)
{   int slope_temp_up = 0;
    int slope_temp_down = 0;
    uint16_t temp_up_pos_max=0;
	uint16_t temp_down_pos_max=0;
    // 压缩数据，10个平均一下
    if (ccd->ccd_state == CCD_OK)
    {
        memset(ccd->ccd_Compress_data, 0, sizeof(ccd->ccd_Compress_data));
        for (int i = 0; i < 100; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                ccd->ccd_Compress_data[i] += ccd->ccd_origin_data[i * 10 + j] / 10;
            }
        }
        ccd->ccd_state = CCD_WAIT;
    }

    // 遍历所有区域找斜率最大点   
    for (int i = 0; i < 100; i++)
    {
        if (i > left_limit && i < right_limit)
        {
            // 找上升沿最高
            if (ccd->ccd_Compress_data[i + 1] >= ccd->ccd_Compress_data[i])
            {
                slope_temp_up = ccd->ccd_Compress_data[i + 1] - ccd->ccd_Compress_data[i];
            }
            // 下降沿最高
            if (ccd->ccd_Compress_data[i + 1] < ccd->ccd_Compress_data[i])
            {
                slope_temp_down = ccd->ccd_Compress_data[i] - ccd->ccd_Compress_data[i + 1];
            }		
            if (slope_temp_up > ccd->slope_up_max)
            {		
				ccd->slope_up_max = slope_temp_up;
			    temp_up_pos_max = i;
            }
            if (slope_temp_down > ccd->slope_down_max)
            {
				ccd->slope_down_max = slope_temp_down;
				temp_down_pos_max = i;
            }
			if(temp_down_pos_max >= temp_up_pos_max)
			{
				ccd->slope_up_max_pos = temp_up_pos_max;
				ccd->slope_down_max_pos = temp_down_pos_max;
			}
        }
    }
 ccd->slope_down_max =0;
 ccd->slope_up_max = 0;
ccd->ccd_slope_max_pos= (ccd->slope_down_max_pos +ccd->slope_up_max_pos)/2;
}
