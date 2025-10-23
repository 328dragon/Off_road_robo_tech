#ifndef __CCD_H
#define __CCD_H

#ifdef __cplusplus
extern "C"
{
#endif
    // 开始
#define ccd_center 102
	#define left_limit 17
#define right_limit 187
#include "spi.h"
#include "string.h"
    enum CCD_State_t
    {
        CCD_WAIT = 0,
        CCD_OK = 1,

    };

    typedef struct CCD_Struct_t
    {
        // 协议层
        GPIO_TypeDef *CCD_DR_IRQ_Port;
        uint16_t CCD_DR_IRQ_Pin;
        GPIO_TypeDef *CCD_SPI_CS_Port;
        uint16_t CCD_SPI_CS_Pin;
        SPI_HandleTypeDef *ccd_hspi;
        uint8_t ccd_dma_rx_buf[1024];
        // 数据层
        uint8_t ccd_origin_data[1000];  // ccd原始数据
        uint8_t ccd_Compress_data[200]; // CCD压缩数据
        uint8_t ccd_state;              // CCD状态
        uint16_t slope_up_max_pos;
        uint16_t slope_down_max_pos;

        // 应用层
        int ccd_slope_max_pos;
    } CCD_t;
    // 外部引用
    void CCD_Init(CCD_t *ccd,
                  GPIO_TypeDef *DR_IRQ_Port, uint16_t _DR_IRQ_Pin, GPIO_TypeDef *CCD_SPI_CS_Port,
                  uint16_t CCD_SPI_CS_Pin,
                  SPI_HandleTypeDef *ccd_hspi);
    void ccd_exti_callback(CCD_t *ccd, uint16_t GPIO_Pin);
    void ccd_spi_rx_cplt_callback(CCD_t *ccd, SPI_HandleTypeDef *hspi);
    void ccd_data_process(CCD_t *ccd);
#ifdef __cplusplus
}
#endif

#endif
