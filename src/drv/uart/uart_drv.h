/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : uart_drv.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : Friday, January 11, 2019
  最近修改   :
  功能描述   : 串口驱动层程序，仅与硬件平台有关
  函数列表   :
  修改历史   :
  1.日    期   : Friday, January 11, 2019
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

#ifndef _UART_DRV_H_
#define _UART_DRV_H_

#include "pubdef.h"

#define RX_LEN (SIZE_2K/2)

/* 串口号枚举 */
typedef enum
{
	UART_COM1 = 0,
	UART_COM2,
	UART_COM3,
	UART_COM4,
	UART_COM5,
	UART_COM6,
	UART_COM7,
	UART_COM_NUM,
}uart_type_enum;

/* 串口中断类型 */
typedef enum
{
	UART_INTR_NONE = 0,
	UART_INTR_TX,
	UART_INTR_RX,
	UART_INTR_TX_RX,
	UART_INTR_IDLE,
}uart_intr_enum;

/*DMA接收结构体 by Zhangxf*/
typedef struct  
{  
  volatile uint8_t  RX_flag:1;        //IDLE receive flag
  volatile uint16_t RX_Size;          //receive length
  uint8_t  RX_pData[RX_LEN]; //DMA receive buffer
}USART_TYPE; 

extern USART_TYPE UwbUart;  
extern USART_TYPE SocketBuff;
extern USART_TYPE UbxUart;
//extern USART_TYPE CarUbxUart;
extern USART_TYPE CacheBuff;


/* 串口驱动结构 */
typedef struct
{	
	uart_type_enum 		uart_com;					/*串口号*/
	UART_HandleTypeDef*	uart_handle;				/*操作对象*/
	U32			 		uart_mode;					/*工作模式*/
	USART_TypeDef* 		uart_id;					/*串口地址*/
	U32					baud_rate;					/*波特率*/
	U32					word_len;					/*字长*/
	U32					stop_bits;					/*停止位数*/
	U32					parity;						/*校验位*/
	U32					hw_flow;					/*硬件流控*/

	/*发送端口配置*/
	GPIO_TypeDef*		gpio_tx_port;				/*GPIO端口*/
	U32					gpio_tx_pin;				/*GPIO PIN*/
	U32					gpio_tx_mode;				/*GPIO工作模式*/
	U32					gpio_tx_pull;				/*GPIO上拉模式*/
	U32					gpio_tx_speed;				/*GPIO速度*/
	U32					gpio_tx_alternate;			/*GPIO复用功能*/

	/*接收端口配置*/
	GPIO_TypeDef*		gpio_rx_port;				/*GPIO端口*/
	U32					gpio_rx_pin;				/*GPIO PIN*/
	U32					gpio_rx_mode;				/*GPIO工作模式*/
	U32					gpio_rx_pull;				/*GPIO上拉模式*/
	U32					gpio_rx_speed;				/*GPIO速度*/
	U32					gpio_rx_alternate;			/*GPIO复用功能*/

	uart_intr_enum 		irq_type;					/*中断类型*/
	IRQn_Type			irq_num;					/*中断号*/		
	U32					prio;						/*中断优先级*/
	U32					sub_prio;					/*中断子优先级*/

	U32					dma_tx_enable;				/*DMA使能*/
	DMA_HandleTypeDef*	dma_tx_handle;				/*DMA操作对象*/
	DMA_Stream_TypeDef*	dma_tx_stream;				/*DMA发送流*/
	U32					dma_tx_request;			/*DMA目标外设*/

	U32					dma_rx_enable;				/*DMA使能*/
	DMA_HandleTypeDef*	dma_rx_handle;				/*DMA操作对象*/
	DMA_Stream_TypeDef*	dma_rx_stream;				/*DMA接收流*/
	U32					dma_rx_request;			/*DMA接收外设*/

	U32 				uart_init_done;			/*串口初始化标志*/

}uart_drv_t;

typedef BOOL (*uart_rx_callback_def)(CHAR_T data);	/*定义串口接收回调函数*/

extern void uart_drv_init(void);		/*串口驱动初始化*/
extern BOOL uart_register_rx_callbak(uart_type_enum com, uart_rx_callback_def cb);	/*注册串口接收回调函数*/
extern void uart_send_data(uart_type_enum com, U8* buf, S32 len);		/*串口发送一串数据*/
extern void uart_set_baudrate(uart_type_enum com, U32 baud);
extern void HAL_UART_IDLECallback(UART_HandleTypeDef *huart);
extern int DMA_Data_send(uart_type_enum com,U8* buf, S32 len);

//改变串口3波特率
extern void USART3_modify_baud_rate(U32 rate);

#endif
/* eof */
