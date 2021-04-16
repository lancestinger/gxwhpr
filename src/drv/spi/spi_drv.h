/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : spi_drv.h
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��6��28��
  ����޸�   :
  ��������   : SPI����ͷ�ļ�
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��6��28��
    ��    ��   : wzq
    �޸�����   : �����ļ�

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
	SPI_HandleTypeDef* 			p_spi_handle;			/*SPI��������*/
	SPI_TypeDef* 				spi_sel;				/*SPIģ��ѡ��*/
	U32							mode;					/*SPIģʽ*/
	U32							direction;				/*���䷽��*/
	U32							data_size;				/*�������ݿ��*/
	U32							nss_sel;				/*Ƭѡ�ź�ѡ��*/
	U32							baud_rate_prescale;	/*�����ʷ�Ƶϵ��*/
	U32							first_bit;				/*���ݴ��俪ʼ��λ����*/
	U32							clk_polarity;			/*ʱ�ӿ���״̬��ƽ����*/

	GPIO_TypeDef*				nss_gpio_port;			/*NSS GPIO�˿�*/
	U32							nss_gpio_pin;			/*NSS GPIO PIN*/
	U32							nss_gpio_alternate;	/*NSS GPIO���ù���*/

	GPIO_TypeDef*				clk_gpio_port;			/*CLK GPIO�˿�*/
	U32							clk_gpio_pin;			/*CLK GPIO PIN*/
	U32							clk_gpio_alternate;	/*CLK GPIO���ù���*/

	GPIO_TypeDef*				miso_gpio_port;		/*MISO GPIO�˿�*/
	U32							miso_gpio_pin;			/*MISO GPIO PIN*/
	U32							miso_gpio_alternate;	/*MISO GPIO���ù���*/

	GPIO_TypeDef*				mosi_gpio_port;		/*MOSI GPIO�˿�*/
	U32							mosi_gpio_pin;			/*MOSI GPIO PIN*/
	U32							mosi_gpio_alternate;	/*MOSI GPIO���ù���*/
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
