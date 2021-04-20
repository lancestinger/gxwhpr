/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : uart_drv.h
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : Friday, January 11, 2019
  ����޸�   :
  ��������   : ������������򣬽���Ӳ��ƽ̨�й�
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : Friday, January 11, 2019
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/

#ifndef _UART_DRV_H_
#define _UART_DRV_H_

#include "pubdef.h"

#define RX_LEN (SIZE_2K/2)

/* ���ں�ö�� */
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

/* �����ж����� */
typedef enum
{
	UART_INTR_NONE = 0,
	UART_INTR_TX,
	UART_INTR_RX,
	UART_INTR_TX_RX,
	UART_INTR_IDLE,
}uart_intr_enum;

/*DMA���սṹ�� by Zhangxf*/
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


/* ���������ṹ */
typedef struct
{	
	uart_type_enum 		uart_com;					/*���ں�*/
	UART_HandleTypeDef*	uart_handle;				/*��������*/
	U32			 		uart_mode;					/*����ģʽ*/
	USART_TypeDef* 		uart_id;					/*���ڵ�ַ*/
	U32					baud_rate;					/*������*/
	U32					word_len;					/*�ֳ�*/
	U32					stop_bits;					/*ֹͣλ��*/
	U32					parity;						/*У��λ*/
	U32					hw_flow;					/*Ӳ������*/

	/*���Ͷ˿�����*/
	GPIO_TypeDef*		gpio_tx_port;				/*GPIO�˿�*/
	U32					gpio_tx_pin;				/*GPIO PIN*/
	U32					gpio_tx_mode;				/*GPIO����ģʽ*/
	U32					gpio_tx_pull;				/*GPIO����ģʽ*/
	U32					gpio_tx_speed;				/*GPIO�ٶ�*/
	U32					gpio_tx_alternate;			/*GPIO���ù���*/

	/*���ն˿�����*/
	GPIO_TypeDef*		gpio_rx_port;				/*GPIO�˿�*/
	U32					gpio_rx_pin;				/*GPIO PIN*/
	U32					gpio_rx_mode;				/*GPIO����ģʽ*/
	U32					gpio_rx_pull;				/*GPIO����ģʽ*/
	U32					gpio_rx_speed;				/*GPIO�ٶ�*/
	U32					gpio_rx_alternate;			/*GPIO���ù���*/

	uart_intr_enum 		irq_type;					/*�ж�����*/
	IRQn_Type			irq_num;					/*�жϺ�*/		
	U32					prio;						/*�ж����ȼ�*/
	U32					sub_prio;					/*�ж������ȼ�*/

	U32					dma_tx_enable;				/*DMAʹ��*/
	DMA_HandleTypeDef*	dma_tx_handle;				/*DMA��������*/
	DMA_Stream_TypeDef*	dma_tx_stream;				/*DMA������*/
	U32					dma_tx_request;			/*DMAĿ������*/

	U32					dma_rx_enable;				/*DMAʹ��*/
	DMA_HandleTypeDef*	dma_rx_handle;				/*DMA��������*/
	DMA_Stream_TypeDef*	dma_rx_stream;				/*DMA������*/
	U32					dma_rx_request;			/*DMA��������*/

	U32 				uart_init_done;			/*���ڳ�ʼ����־*/

}uart_drv_t;

typedef BOOL (*uart_rx_callback_def)(CHAR_T data);	/*���崮�ڽ��ջص�����*/

extern void uart_drv_init(void);		/*����������ʼ��*/
extern BOOL uart_register_rx_callbak(uart_type_enum com, uart_rx_callback_def cb);	/*ע�ᴮ�ڽ��ջص�����*/
extern void uart_send_data(uart_type_enum com, U8* buf, S32 len);		/*���ڷ���һ������*/
extern void uart_set_baudrate(uart_type_enum com, U32 baud);
extern void HAL_UART_IDLECallback(UART_HandleTypeDef *huart);
extern int DMA_Data_send(uart_type_enum com,U8* buf, S32 len);

//�ı䴮��3������
extern void USART3_modify_baud_rate(U32 rate);

#endif
/* eof */
