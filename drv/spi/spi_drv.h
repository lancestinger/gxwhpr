/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : spi_drv.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年6月28日
  最近修改   :
  功能描述   : SPI驱动头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年6月28日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

#ifndef __SPI_DRV_H_
#define __SPI_DRV_H_

#include "pubdef.h"

typedef enum
{
	SPI_1 = 0,
	SPI_2,
	//SPI_3,
	SPI_4,
	SPI_NUM,
}spi_type_enum;

typedef struct
{
	SPI_HandleTypeDef* 			p_spi_handle;			/*SPI操作对象*/
	SPI_TypeDef* 				spi_sel;				/*SPI模块选择*/
	U32							mode;					/*SPI模式*/
	U32							direction;				/*传输方向*/
	U32							data_size;				/*传输数据宽度*/
	U32							nss_sel;				/*片选信号选择*/
	U32							baud_rate_prescale;	/*波特率分频系数*/
	U32							first_bit;				/*数据传输开始的位类型*/
	U32							clk_polarity;			/*时钟空闲状态电平极性*/

	GPIO_TypeDef*				nss_gpio_port;			/*NSS GPIO端口*/
	U32							nss_gpio_pin;			/*NSS GPIO PIN*/
	U32							nss_gpio_alternate;	/*NSS GPIO复用功能*/

	GPIO_TypeDef*				clk_gpio_port;			/*CLK GPIO端口*/
	U32							clk_gpio_pin;			/*CLK GPIO PIN*/
	U32							clk_gpio_alternate;	/*CLK GPIO复用功能*/

	GPIO_TypeDef*				miso_gpio_port;		/*MISO GPIO端口*/
	U32							miso_gpio_pin;			/*MISO GPIO PIN*/
	U32							miso_gpio_alternate;	/*MISO GPIO复用功能*/

	GPIO_TypeDef*				mosi_gpio_port;		/*MOSI GPIO端口*/
	U32							mosi_gpio_pin;			/*MOSI GPIO PIN*/
	U32							mosi_gpio_alternate;	/*MOSI GPIO复用功能*/
}spi_drv_t;

//#define AD936X_SPI	SPI_1

extern SPI_HandleTypeDef spi1_handle, spi2_handle, spi3_handle, spi4_handle;


#define SPI1_CS_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define SPI1_CS_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)

#define SPI2_CS_HIGH	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET)
#define SPI2_CS_LOW	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET)

extern void spi_drv_init(void);
extern U8 spi_read_write_byte(spi_type_enum type, U8 data);
extern void spi_write_buffer(spi_type_enum type, IN U8* buf, IN U32 len);
extern void spi_read_buffer(spi_type_enum type, INOUT U8* buf, IN U32 len);
extern void spi_set_datasize(spi_type_enum type, U8 size);
extern void spi_set_clkphase(spi_type_enum type, U32 clkphase);
extern U8 SPI2_ReadWriteByte(U8 TxData);
extern void spi_set_baudrate(spi_type_enum type, U32 prescale);
extern void spi_set_enable(spi_type_enum type);
extern void spi_set_disable(spi_type_enum type);
extern void spi_fast_write_buffer(spi_type_enum type, IN U8* buf, IN U32 len);
extern void spi_fast_read_buffer(spi_type_enum type, INOUT U8* buf, IN U32 len);
#endif
/*eof*/
