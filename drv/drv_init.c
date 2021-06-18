/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : drv_init.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年5月29日
  最近修改   :
  功能描述   : 驱动初始化文件
  函数列表   :
              _fs_init
  修改历史   :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

/*------------------------------头文件------------------------------*/
#include "w25qxx/w25qxx_drv.h"
#include "uart/uart_drv.h"
#include "adc/adc_drv.h"
#include "spi/spi_drv.h"
#include "socket/socket_drv.h"
#include "uwb_post/uwb_post.h"

#include "gpio/gpio_drv.h"
#include "project_def.h"
/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/

/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
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
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/


/*****************************************************************************
 函 数 名  : _fs_init
 功能描述  : 文件系统初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 新生成函数

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
		NOTE_PRINT(("文件系统初始化OK!!!!!!\r\n"));
		free_len = ffree("F0:")/(1024*1024.0);
		GLOBAL_TRACE(("FLASH剩余空间: %.2f MB.\r\n", free_len));
	}
	else
	{
		ERR_PRINT(("文件系统初始化失败!!!!!!\r\n"));
	}
}

/*****************************************************************************
 函 数 名  : _mx_gpio_init
 功能描述  : 使能GPIO时钟
 输入参数  : void  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 新生成函数

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
 函 数 名  : drv_init
 功能描述  : 驱动初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 新生成函数

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
