/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : drv_init.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��5��29��
  ����޸�   :
  ��������   : ������ʼ���ļ�
  �����б�   :
              _fs_init
  �޸���ʷ   :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/

/*------------------------------ͷ�ļ�------------------------------*/
#include "w25qxx/w25qxx_drv.h"
#include "uart/uart_drv.h"
#include "adc/adc_drv.h"
#include "spi/spi_drv.h"
#include "socket/socket_drv.h"
#include "uwb_post/uwb_post.h"

#include "gpio/gpio_drv.h"
#include "project_def.h"
/*------------------------------ͷ�ļ�------------------------------*/


/*------------------------------�ļ���------------------------------*/

/*------------------------------�ļ���------------------------------*/


/*------------------------------�ļ�����------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */
#endif

ETH_HandleTypeDef heth;
/*------------------------------�ļ�����------------------------------*/


/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/


/*****************************************************************************
 �� �� ��  : _fs_init
 ��������  : �ļ�ϵͳ��ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
static void _fs_init(void)
{
	static F32 free_len = 0;
	
	if (finit("F0:") == fsOK) 
	{
		if( (fmount("F0:")) != fsOK)
		{
			w25qxx_erase_fs_sector();
			if(fmount("F0:") == fsOK)
			{
			}
		}
		NOTE_PRINT(("�ļ�ϵͳ��ʼ��OK!!!!!!\r\n"));
		free_len = ffree("F0:")/(1024*1024.0);
		GLOBAL_TRACE(("FLASHʣ��ռ�: %.2f MB.\r\n", free_len));
	}
	else
	{
		ERR_PRINT(("�ļ�ϵͳ��ʼ��ʧ��!!!!!!\r\n"));
	}
}

/*****************************************************************************
 �� �� ��  : _mx_gpio_init
 ��������  : ʹ��GPIOʱ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
static void _mx_gpio_init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
}

/*****************************************************************************
 �� �� ��  : drv_init
 ��������  : ������ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void drv_init(void)
{
	_mx_gpio_init();	/*must be first*/
	uart_drv_init();
	sys_debug_init();
	gpio_drv_init();
	spi_drv_init();
	_fs_init();
	sys_para_init();
	adc_drv_init();
	netInitialize();
	socket_udp_drv_init(SOCKET_3);
	gpio_enable_irq();
	evt_id = osEventFlagsNew(NULL);

}
