/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : uart_drv.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : Friday, January 11, 2019
  最近修改   :
  功能描述   : 串口驱动文件
  函数列表   :
              _cfg_uart_tx_dma
              uart_drv_init
              _uart_rcv_data
              uart_register_rx_callbak
              uart_send_data
              USART1_IRQHandler
  修改历史   :
  1.日    期   : Friday, January 11, 2019
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

#include "drv/uart/uart_drv.h"
#include "comm/project_def.h"
#include "rngbuf/rngbuf.h"

#define UART_INIT_NUM	5	//需要初始化串口的数目

static UART_HandleTypeDef 	uart1_handle, uart3_handle,uart4_handle,uart5_handle,uart6_handle,uart7_handle;
static DMA_HandleTypeDef	uart1_dma_tx_handle, uart3_dma_tx_handle, uart4_dma_tx_handle, uart5_dma_tx_handle,uart6_dma_tx_handle,uart7_dma_tx_handle;
static DMA_HandleTypeDef	uart1_dma_rx_handle, uart3_dma_rx_handle, uart4_dma_rx_handle, uart5_dma_rx_handle,uart6_dma_rx_handle,uart7_dma_rx_handle;
static uart_rx_callback_def uart_rx_callback[UART_COM_NUM];

USART_TYPE UwbUart = {.RX_flag=0,};
USART_TYPE SocketBuff = {.RX_flag=0,};
USART_TYPE UDPBuff = {.RX_flag=0,};
NMEA_TYPE NMEA_buf = {.RX_flag=0,};
NMEA_TYPE RMC_buf = {.RX_flag=0,};
USART_TYPE CacheBuff = {.RX_flag=0,};

//static U8 rx_buf = 0;

static const uart_drv_t uart_drv_cfg[UART_INIT_NUM] =
{
	/*串口1,备用调试打印串口*/
	{
		/*基本配置*/
		UART_COM1,
		&uart1_handle,
		UART_MODE_TX_RX,
		USART1,
		115200,
		UART_WORDLENGTH_8B,
		UART_STOPBITS_1,
		UART_PARITY_NONE,
		UART_HWCONTROL_NONE,

		/*GPIO TX配置*/
		GPIOA,
		GPIO_PIN_9,
		GPIO_MODE_AF_PP,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF7_USART1,

		/*GPIO RX配置*/
		GPIOA,
		GPIO_PIN_10,
		GPIO_MODE_AF_PP,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF7_USART1,

		/*中断配置*/
		UART_INTR_RX,
		USART1_IRQn,
		11,
		0,

		/*发送DMA配置*/
		UART_DMA_TX_ENABLE,
		&uart1_dma_tx_handle,
		DMA1_Stream3,
		DMA_REQUEST_USART1_TX,

		///接收DMA配置
		UART_DMA_RX_DISABLE,
		&uart1_dma_rx_handle,
		DMA1_Stream2,
		DMA_REQUEST_USART1_RX,

		TRUE,
	},

	/*串口3,调试打印串口*/
	{
		/*基本配置*/
		UART_COM3,
		&uart3_handle,
		UART_MODE_TX_RX,
		USART3,
        230400,
		UART_WORDLENGTH_8B,
		UART_STOPBITS_1,
		UART_PARITY_NONE,
		UART_HWCONTROL_NONE,

		/*GPIO TX配置*/
		GPIOB,
		GPIO_PIN_10,
		GPIO_MODE_AF_PP,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF7_USART3,

		/*GPIO RX配置*/
		GPIOB,
		GPIO_PIN_11,
		GPIO_MODE_AF_PP,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF7_USART3,

		/*中断配置*/
		UART_INTR_RX,
		USART3_IRQn,
        5,
        1,
      
		/*发送DMA配置*/
		UART_DMA_TX_ENABLE,
		&uart3_dma_tx_handle,
		DMA1_Stream5,
		DMA_REQUEST_USART3_TX,

	  	/*接收DMA配置*/
		UART_DMA_RX_DISABLE,
		&uart3_dma_rx_handle,
		DMA1_Stream4,
		DMA_REQUEST_USART3_RX,
		
		TRUE,
	},

	/*串口4,ZED_GNSS通信串口*/
	{
		/*基本配置*/
		UART_COM4,
		&uart4_handle,
		UART_MODE_TX_RX,
		UART4,
		115200,
		UART_WORDLENGTH_8B,
		UART_STOPBITS_1,
		UART_PARITY_NONE,
		UART_HWCONTROL_NONE,
	
		/*GPIO TX配置*/
		GPIOA,
		GPIO_PIN_12,
		GPIO_MODE_AF_PP,
		GPIO_NOPULL,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF6_UART4,
	
		/*GPIO RX配置*/
		GPIOI,
		GPIO_PIN_9,
		GPIO_MODE_AF_PP,
		GPIO_NOPULL,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF8_UART4,
	
		/*中断配置*/
		UART_INTR_RX,
		UART4_IRQn,
		3,
		1,
	
		/*发送DMA配置*/
		UART_DMA_TX_ENABLE,
		&uart4_dma_tx_handle,
		DMA1_Stream7,
		DMA_REQUEST_UART4_TX,
	
		/*接收DMA配置*/
		UART_DMA_RX_DISABLE,
		&uart4_dma_rx_handle,
		DMA1_Stream6,
		DMA_REQUEST_UART4_RX,
		
		TRUE,
	},

	/*串口5*/
	{
		/*基本配置*/
		UART_COM5,
		&uart5_handle,
		UART_MODE_TX_RX,
		UART5,
		9600,
		UART_WORDLENGTH_8B,
		UART_STOPBITS_1,
		UART_PARITY_NONE,
		UART_HWCONTROL_NONE,

		/*GPIO TX配置*/
		GPIOC,
		GPIO_PIN_12,
		GPIO_MODE_AF_PP,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF8_UART5,

		/*GPIO RX配置*/
		GPIOD,
		GPIO_PIN_2,
		GPIO_MODE_AF_PP,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF8_UART5,

		/*中断配置*/
		UART_INTR_NONE,
		UART5_IRQn,
		10,
		0,

		/*发送DMA配置*/
		UART_DMA_TX_DISABLE,
		&uart5_dma_tx_handle,
		DMA1_Stream1,
		DMA_REQUEST_UART5_TX,

		/*接收DMA配置*/
		UART_DMA_RX_DISABLE,
		&uart5_dma_rx_handle,
		DMA1_Stream0,
		DMA_REQUEST_UART5_RX,

		TRUE,
	},

	/*串口6,转换RS232调试串口*/
	{
		/*基本配置*/
		UART_COM6,
		&uart6_handle,
		UART_MODE_TX_RX,
		USART6,
		115200,
		UART_WORDLENGTH_8B,
		UART_STOPBITS_1,
		UART_PARITY_NONE,
		UART_HWCONTROL_NONE,

		/*GPIO TX配置*/
		GPIOC,
		GPIO_PIN_6,
		GPIO_MODE_AF_PP,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF7_USART6,

		/*GPIO RX配置*/
		GPIOC,
		GPIO_PIN_7,
		GPIO_MODE_AF_PP,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF7_USART6,

		/*中断配置*/
		UART_INTR_RX,
		USART6_IRQn,
		3,
		2,

		/*发送DMA配置*/
		UART_DMA_TX_ENABLE,
		&uart6_dma_tx_handle,
		DMA2_Stream1,
		DMA_REQUEST_USART6_TX,

		/*接收DMA配置*/
		UART_DMA_RX_DISABLE,
		&uart6_dma_rx_handle,
		DMA2_Stream0,
		DMA_REQUEST_USART6_RX,
		
		TRUE,
	},

#if 0		
	/*串口7,GNSS通信串口*/
	{
		/*基本配置*/
		UART_COM7,
		&uart7_handle,
		UART_MODE_TX_RX,
		UART7,
		115200,
		UART_WORDLENGTH_8B,
		UART_STOPBITS_1,
		UART_PARITY_NONE,
		UART_HWCONTROL_NONE,
	
		/*GPIO TX配置*/
		GPIOF,
		GPIO_PIN_7,
		GPIO_MODE_AF_PP,
		GPIO_NOPULL,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF7_UART7,
	
		/*GPIO RX配置*/
		GPIOF,
		GPIO_PIN_6,
		GPIO_MODE_AF_PP,
		GPIO_NOPULL,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_AF7_UART7,
	
		/*中断配置*/
		UART_INTR_RX,
		UART7_IRQn,
		3,
		3,
	
		/*发送DMA配置*/
		UART_DMA_TX_ENABLE,
		&uart7_dma_tx_handle,
		DMA1_Stream7,
		DMA_REQUEST_UART7_TX,
	
		/*接收DMA配置*/
		UART_DMA_RX_DISABLE,
		&uart7_dma_rx_handle,
		DMA1_Stream6,
		DMA_REQUEST_UART7_RX,
		
		TRUE,
	},
#endif
};

/*****************************************************************************
 函 数 名  : _cfg_uart_dma
 功能描述  : 串口DMA发送配置
 输入参数  : DMA_HandleTypeDef* uart_tx_dma_handle  
             UART_HandleTypeDef* uart_handle        
             DMA_Stream_TypeDef *dma_stream         
             U32 chn                                
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : Friday, January 11, 2019
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static void _cfg_uart_dma(DMA_HandleTypeDef* uart_tx_dma_handle, UART_HandleTypeDef* uart_handle, DMA_Stream_TypeDef *dma_stream, U32 dma_request)
{ 

	if((U32)dma_stream>(U32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能	
	}else 
	{
        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
	}
    
    __HAL_LINKDMA(uart_handle, hdmatx, *uart_tx_dma_handle);    //将DMA与USART1联系起来(发送DMA)
    
    //Tx DMA配置
    uart_tx_dma_handle->Instance=dma_stream;                            //数据流选择
    uart_tx_dma_handle->Init.Request=dma_request;                       //通道选择
    uart_tx_dma_handle->Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
    uart_tx_dma_handle->Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    uart_tx_dma_handle->Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    uart_tx_dma_handle->Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    uart_tx_dma_handle->Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    uart_tx_dma_handle->Init.Mode=DMA_NORMAL;                            //外设流控模式
    uart_tx_dma_handle->Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    uart_tx_dma_handle->Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    uart_tx_dma_handle->Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    uart_tx_dma_handle->Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    uart_tx_dma_handle->Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
    
    HAL_DMA_DeInit(uart_tx_dma_handle);   
    HAL_DMA_Init(uart_tx_dma_handle);
} 

/*****************************************************************************
 Prototype    : HAL_UART_MspInit
 Description  : HAL串口初始化会调用该函数
 Input        : UART_HandleTypeDef *huart  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/5/29
    Author       : wzq
    Modification : Created function

*****************************************************************************/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
	}
	else if(huart->Instance == USART3)
	{
		__HAL_RCC_USART3_CLK_ENABLE();			//使能USART3时钟
	}
	else if(huart->Instance == UART4)
	{
		__HAL_RCC_UART4_CLK_ENABLE();			//使能UART7时钟
	}
	else if(huart->Instance == UART5)
	{
		__HAL_RCC_UART5_CLK_ENABLE();			//使能USART5时钟
	}
	else if(huart->Instance == USART6)
	{
		__HAL_RCC_USART6_CLK_ENABLE();			//使能USART6时钟
	}
	else if(huart->Instance == UART7)
	{
		__HAL_RCC_UART7_CLK_ENABLE();			//使能UART7时钟
	}


}


/*****************************************************************************
 函 数 名  : uart_drv_init
 功能描述  : 初始化串口驱动
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : Friday, January 11, 2019
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void uart_drv_init(void)
{
	U8 i = 0;
	GPIO_InitTypeDef GPIO_Initure;

	GLOBAL_MEMSET(uart_rx_callback, 0x0, sizeof(uart_rx_callback));
	for(i = 0; i < UART_INIT_NUM; i++)
	{
		GLOBAL_MEMSET(uart_drv_cfg[i].uart_handle, 0, sizeof(UART_HandleTypeDef));
		/*发送端口配置*/
		GPIO_Initure.Pin = uart_drv_cfg[i].gpio_tx_pin;
		GPIO_Initure.Mode = uart_drv_cfg[i].gpio_tx_mode;
		GPIO_Initure.Pull = uart_drv_cfg[i].gpio_tx_pull;
		GPIO_Initure.Speed = uart_drv_cfg[i].gpio_tx_speed;
		GPIO_Initure.Alternate = uart_drv_cfg[i].gpio_tx_alternate;
		HAL_GPIO_Init(uart_drv_cfg[i].gpio_tx_port,&GPIO_Initure);

		/*接收端口配置*/
		GPIO_Initure.Pin = uart_drv_cfg[i].gpio_rx_pin;
		GPIO_Initure.Mode = uart_drv_cfg[i].gpio_rx_mode;
		GPIO_Initure.Pull = uart_drv_cfg[i].gpio_rx_pull;
		GPIO_Initure.Speed = uart_drv_cfg[i].gpio_rx_speed;
		GPIO_Initure.Alternate = uart_drv_cfg[i].gpio_rx_alternate;
		HAL_GPIO_Init(uart_drv_cfg[i].gpio_rx_port,&GPIO_Initure);

		/*基本配置*/
		uart_drv_cfg[i].uart_handle->Instance = uart_drv_cfg[i].uart_id;
		uart_drv_cfg[i].uart_handle->Init.BaudRate = uart_drv_cfg[i].baud_rate;
		uart_drv_cfg[i].uart_handle->Init.WordLength = uart_drv_cfg[i].word_len;
		uart_drv_cfg[i].uart_handle->Init.StopBits = uart_drv_cfg[i].stop_bits;
		uart_drv_cfg[i].uart_handle->Init.Parity = uart_drv_cfg[i].parity;
		uart_drv_cfg[i].uart_handle->Init.HwFlowCtl = uart_drv_cfg[i].hw_flow;
		uart_drv_cfg[i].uart_handle->Init.Mode = uart_drv_cfg[i].uart_mode;
		HAL_UART_Init(uart_drv_cfg[i].uart_handle);
		
		/*中断配置*/
		if(uart_drv_cfg[i].irq_type != UART_INTR_NONE)
		{
			switch(uart_drv_cfg[i].irq_type)
			{
				case UART_INTR_TX:
				{
					__HAL_UART_ENABLE_IT(uart_drv_cfg[i].uart_handle,UART_IT_TXE);	
					break;
				}
				case UART_INTR_RX:
				{
					__HAL_UART_ENABLE_IT(uart_drv_cfg[i].uart_handle,UART_IT_RXNE);	
					break;
				}
				case UART_INTR_TX_RX:
				{
					__HAL_UART_ENABLE_IT(uart_drv_cfg[i].uart_handle,UART_IT_TXE);	
					__HAL_UART_ENABLE_IT(uart_drv_cfg[i].uart_handle,UART_IT_RXNE);	
					break;
				}
				case UART_INTR_IDLE:
				{
					//__HAL_UART_ENABLE_IT(uart_drv_cfg[i].uart_handle,UART_IT_IDLE);
					break;
				}
				default:
					break;
			}
			
			__HAL_UART_ENABLE_IT(uart_drv_cfg[i].uart_handle, UART_IT_ERR);
			HAL_NVIC_EnableIRQ(uart_drv_cfg[i].irq_num);
			HAL_NVIC_SetPriority(uart_drv_cfg[i].irq_num, uart_drv_cfg[i].prio,uart_drv_cfg[i].sub_prio);	
		}

		
		/*if((uart_drv_cfg[i].irq_type == UART_INTR_RX) || (uart_drv_cfg[i].irq_type == UART_INTR_TX_RX))
		{
			//HAL_UART_Receive_IT(uart_drv_cfg[i].uart_handle, (U8 *)rx_buf, 1);//
		}*/
		/*DMA发送配置*/
		if(uart_drv_cfg[i].dma_tx_enable == UART_DMA_TX_ENABLE)
		{
			_cfg_uart_dma(uart_drv_cfg[i].dma_tx_handle, uart_drv_cfg[i].uart_handle, uart_drv_cfg[i].dma_tx_stream, uart_drv_cfg[i].dma_tx_request);
		}
		
		/*DMA接收配置*/
		if(uart_drv_cfg[i].dma_rx_enable == UART_DMA_RX_ENABLE)
		{
			//_cfg_uart_dma(uart_drv_cfg[i].dma_rx_handle, uart_drv_cfg[i].uart_handle, uart_drv_cfg[i].dma_rx_stream, uart_drv_cfg[i].dma_rx_request);
		}
		//DMA_Data_send(UART_COM1, "hello\r\n", 7);
		//HAL_UART_Receive_DMA(&uart3_handle, UwbUart.RX_pData, RX_LEN);
	}
	GLOBAL_PRINT((ANSI_DARK_RED"\r\n\r\n\r\n%s system is power on!!!!!\r\n"ANSI_NONE"", PROJECT_NAME));
	GLOBAL_PRINT(("串口初始化完成!!!!!\r\n"))
}

/*****************************************************************************
 函 数 名  : uart_register_rx_callbak
 功能描述  : 串口接收回调注册
 输入参数  : void (*fun)(CHAR_T c)  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : Friday, January 11, 2019
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
BOOL uart_register_rx_callbak(uart_type_enum com, uart_rx_callback_def cb)
{
	BOOL ret = FALSE;

	if(uart_rx_callback[com] == NULL)
	{
		uart_rx_callback[com] = cb;
	}
	
	return ret;
}

/*****************************************************************************
 函 数 名  : _uart_rcv_data
 功能描述  : 串口接收数据
 输入参数  : uart_type_enum com  
             U8 data             
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : Friday, January 11, 2019
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static BOOL _uart_rcv_data(uart_type_enum com, U8 data)
{
	if(uart_rx_callback[com])
	{
		return uart_rx_callback[com](data);
	}
	return FALSE;
}

/*****************************************************************************
 函 数 名  : uart_send_data
 功能描述  : 串口发送数据
 输入参数  : uart_type_enum com  
             U8* buf             
             U32 len             
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : Friday, January 11, 2019
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void uart_send_data(uart_type_enum com, U8* buf, S32 len)
{
	S32 i = 0, j = 0;

	for(i = 0; i < UART_INIT_NUM; i++)
	{
		if((uart_drv_cfg[i].uart_com == com) && (buf)&& (uart_drv_cfg[i].uart_init_done == TRUE))
		{
			if(uart_drv_cfg[i].dma_tx_enable == UART_DMA_TX_ENABLE)
			{
				HAL_UART_Transmit_DMA(uart_drv_cfg[i].uart_handle, (U8*)buf, len);
				//delay_ms((len*10.0*1000.0*5.0)/(uart_drv_cfg[i].baud_rate)+1);
				while(1)
				{
					if(__HAL_DMA_GET_FLAG(uart_drv_cfg[i].dma_tx_handle,DMA_FLAG_TCIF3_7))
					{
						__HAL_DMA_CLEAR_FLAG(uart_drv_cfg[i].dma_tx_handle, DMA_FLAG_TCIF3_7);//清除DMA传输完成标志
						HAL_UART_DMAStop(uart_drv_cfg[i].uart_handle);      //传输完成以后关闭串口DMA
						break;
					}
					delay_ms(1);
				}
				
			}
			else
			{
				for(j = 0; j < len; j++)
				{
					while((uart_drv_cfg[i].uart_id->ISR&0X40) == 0);//循环发送,直到发送完毕   
					uart_drv_cfg[i].uart_id->TDR = buf[j];   
				}
			}
			break;
		}
	}
}

int DMA_Data_send(uart_type_enum com, U8* buf, S32 len)
{
	S32 i = 0, j = 0;
	static U16 Over_tim=0;

	for(i = 0; i < UART_INIT_NUM; i++)
	{
		if((uart_drv_cfg[i].uart_com == com) && (buf)&& (uart_drv_cfg[i].uart_init_done == TRUE))
		{
			if(uart_drv_cfg[i].dma_tx_enable == UART_DMA_TX_ENABLE)
			{
				HAL_UART_Transmit_DMA(uart_drv_cfg[i].uart_handle, (U8*)buf, len);
				//delay_ms((len*10.0*1000.0*5.0)/(uart_drv_cfg[i].baud_rate)+1);
				while(1)
				{
					if(__HAL_DMA_GET_FLAG(uart_drv_cfg[i].dma_tx_handle,DMA_FLAG_TCIF1_5))
					{
						__HAL_DMA_CLEAR_FLAG(uart_drv_cfg[i].dma_tx_handle, DMA_FLAG_TCIF1_5);//清除DMA传输完成标志
						HAL_UART_DMAStop(uart_drv_cfg[i].uart_handle);      //传输完成以后关闭串口DMA
						break;
					}
					Over_tim++;
					delay_ms(10);
					if(Over_tim >=400)
					{
						break;
					}
					
				}
				Over_tim = 0;
			}
			else
			{
				for(j = 0; j < len; j++)
				{
					while((uart_drv_cfg[i].uart_id->ISR&0X40) == 0);//循环发送,直到发送完毕   
					uart_drv_cfg[i].uart_id->TDR = buf[j];	 
				}
			}
			break;
		}
	}
	return len;
}


/*****************************************************************************
 函 数 名  : uart_set_baudrate
 功能描述  : 设置串口波特率
 输入参数  : uart_type_enum com  
             U32 baud            
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月6日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void uart_set_baudrate(uart_type_enum com, U32 baud)
{
	U8 i = 0;

	for(i = 0; i < UART_INIT_NUM; i++)
	{
		if(uart_drv_cfg[i].uart_com == com)
		{
			//HAL_UART_DeInit(uart_drv_cfg[i].uart_handle);
			uart_drv_cfg[i].uart_handle->Init.BaudRate = baud;
			HAL_UART_Init(uart_drv_cfg[i].uart_handle);
			break;
		}
	}
}

//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 

	if((__HAL_UART_GET_FLAG(&uart1_handle,UART_FLAG_RXNE)!=RESET))
	{
		U8 temp8 = USART1->RDR;
		_uart_rcv_data(UART_COM1, temp8);
		//__HAL_UART_ENABLE_IT(&UART1_Handler,UART_IT_IDLE);		//开启空闲中断
	}
	else if((__HAL_UART_GET_FLAG(&uart1_handle,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_DISABLE_IT(&uart1_handle,UART_IT_IDLE);		//关闭空闲中断
	}

	if((__HAL_UART_GET_FLAG(&uart1_handle,UART_FLAG_ORE)!=RESET))
	{
		U8 rcv_char = 0;
		__HAL_UART_CLEAR_IT(&uart1_handle,UART_FLAG_ORE);
		rcv_char = USART1->ISR;		//出现溢出错误时，需要回读一下ISR和RDR寄存器
		rcv_char = USART1->RDR;
		rcv_char = rcv_char;
	}
	HAL_UART_IRQHandler(&uart1_handle);
	
} 
//串口3中断服务程序
void USART3_IRQHandler(void)                	
{ 
	if((__HAL_UART_GET_FLAG(&uart3_handle,UART_FLAG_RXNE)!=RESET))
		{
			U8 temp8 = USART3->RDR;
			_uart_rcv_data(UART_COM3, temp8);
	
			//__HAL_UART_ENABLE_IT(&UART1_Handler,UART_IT_IDLE);		//开启空闲中断
		}
		else if((__HAL_UART_GET_FLAG(&uart3_handle,UART_FLAG_IDLE)!=RESET))
		{
			__HAL_UART_DISABLE_IT(&uart3_handle,UART_IT_IDLE);		//关闭空闲中断
		}
	
		if((__HAL_UART_GET_FLAG(&uart3_handle,UART_FLAG_ORE)!=RESET))
		{
			U8 rcv_char = 0;
			__HAL_UART_CLEAR_IT(&uart3_handle,UART_FLAG_ORE);
			rcv_char = USART3->ISR;		//出现溢出错误时，需要回读一下ISR和RDR寄存器
			rcv_char = USART3->RDR;
			rcv_char = rcv_char;
		}
		HAL_UART_IRQHandler(&uart3_handle);


} 

void UART4_IRQHandler(void)                	
{ 
	if((__HAL_UART_GET_FLAG(&uart4_handle,UART_FLAG_RXNE)!=RESET))
	{
		U8 temp8 = UART4->RDR;
		_uart_rcv_data(UART_COM4, temp8);
		//GLOBAL_PRINT(("\r\nGET IN UART4_IRQHandler!!!!\r\n"));
		//__HAL_UART_ENABLE_IT(&UART1_Handler,UART_IT_IDLE);		//开启空闲中断
	}
	else if((__HAL_UART_GET_FLAG(&uart4_handle,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_DISABLE_IT(&uart4_handle,UART_IT_IDLE);		//关闭空闲中断
	}

	if((__HAL_UART_GET_FLAG(&uart4_handle,UART_FLAG_ORE)!=RESET))
	{
		U8 rcv_char = 0;
		__HAL_UART_CLEAR_IT(&uart4_handle,UART_FLAG_ORE);
		rcv_char = UART4->ISR;		//出现溢出错误时，需要回读一下ISR和RDR寄存器
		rcv_char = UART4->RDR;
		rcv_char = rcv_char;
	}
	HAL_UART_IRQHandler(&uart4_handle);
	
} 


//串口5中断服务程序
void UART5_IRQHandler(void)                	
{ 

	if((__HAL_UART_GET_FLAG(&uart5_handle,UART_FLAG_RXNE)!=RESET))
	{
		U8 temp8 = UART5->RDR;
		_uart_rcv_data(UART_COM5, temp8);
		//GLOBAL_PRINT(("%c",temp8));
		//__HAL_UART_ENABLE_IT(&UART1_Handler,UART_IT_IDLE);		//开启空闲中断
	}
	else if((__HAL_UART_GET_FLAG(&uart5_handle,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_DISABLE_IT(&uart5_handle,UART_IT_IDLE);		//关闭空闲中断
	}

	if((__HAL_UART_GET_FLAG(&uart5_handle,UART_FLAG_ORE)!=RESET))
	{
		U8 rcv_char = 0;
		__HAL_UART_CLEAR_IT(&uart5_handle,UART_FLAG_ORE);
		rcv_char = UART5->ISR;		//出现溢出错误时，需要回读一下ISR和RDR寄存器
		rcv_char = UART5->RDR;
		rcv_char = rcv_char;
	}
	HAL_UART_IRQHandler(&uart5_handle);
	
} 

//串口6中断服务程序
void USART6_IRQHandler(void)                	
{ 
	extern RNGBUF_t * receiver_rngbuf;

	if((__HAL_UART_GET_FLAG(&uart6_handle,UART_FLAG_RXNE)!=RESET))
	{
		//GLOBAL_PRINT(("UBLOX data GOT!\r\n"));
		U8 temp8 = USART6->RDR;
		_uart_rcv_data(UART_COM6, temp8);
		
        if(g_dbg_out_usart_model==DBG_OUT_USART_RECEIVER_MODEL)
        {
            if(rng_get_free_bytes(receiver_rngbuf) >= 1)
            {
                rng_put_buf(receiver_rngbuf, (char*)&temp8, 1);
            }
        }

		//__HAL_UART_ENABLE_IT(&UART1_Handler,UART_IT_IDLE);		//开启空闲中断
	}
	else if((__HAL_UART_GET_FLAG(&uart6_handle,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_DISABLE_IT(&uart6_handle,UART_IT_IDLE);		//关闭空闲中断
	}

	if((__HAL_UART_GET_FLAG(&uart6_handle,UART_FLAG_ORE)!=RESET))
	{
		U8 rcv_char = 0;
		__HAL_UART_CLEAR_IT(&uart6_handle,UART_FLAG_ORE);
		rcv_char = USART6->ISR;		//出现溢出错误时，需要回读一下ISR和RDR寄存器
		rcv_char = USART6->RDR;
		rcv_char = rcv_char;
	}
	HAL_UART_IRQHandler(&uart6_handle);
	
} 


#if 0
void UART7_IRQHandler(void)                	
{ 
	if((__HAL_UART_GET_FLAG(&uart7_handle,UART_FLAG_RXNE)!=RESET))
	{
		U8 temp8 = UART7->RDR;
		_uart_rcv_data(UART_COM7, temp8);

		//__HAL_UART_ENABLE_IT(&UART1_Handler,UART_IT_IDLE);		//开启空闲中断
	}
	else if((__HAL_UART_GET_FLAG(&uart7_handle,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_DISABLE_IT(&uart7_handle,UART_IT_IDLE);		//关闭空闲中断
	}

	if((__HAL_UART_GET_FLAG(&uart7_handle,UART_FLAG_ORE)!=RESET))
	{
		U8 rcv_char = 0;
		__HAL_UART_CLEAR_IT(&uart7_handle,UART_FLAG_ORE);
		rcv_char = UART7->ISR;		//出现溢出错误时，需要回读一下ISR和RDR寄存器
		rcv_char = UART7->RDR;
		rcv_char = rcv_char;
	}
	HAL_UART_IRQHandler(&uart7_handle);
	
} 
#endif

void HAL_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	//uint32_t temp;
/*	
	if(huart->Instance == USART3){
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET){ 	
			//__HAL_DMA_DISABLE(huart->hdmarx);
			HAL_UART_AbortReceive(huart);
			
			UwbUart.RX_flag = 1;
			__HAL_UART_CLEAR_IDLEFLAG(huart);//qingchubiaozhiwei 
			UwbUart.RX_Size = RX_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

			HAL_UART_Receive_DMA(huart, UwbUart.RX_pData,RX_LEN);
		}
	}
	*/
/*
	if(huart->Instance == UART5){		
	
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET){ 	
			//__HAL_DMA_DISABLE(huart->hdmarx);
			HAL_UART_AbortReceive(huart);
			
			SimUart.RX_flag = 1;
			__HAL_UART_CLEAR_IDLEFLAG(huart);//qingchubiaozhiwei 
			SimUart.RX_Size = RX_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

			HAL_UART_Receive_DMA(huart, SimUart.RX_pData,RX_LEN);
		}
	}
*/
	/*
	if(huart->Instance == UART7)
	{		
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET){ 	
			//__HAL_DMA_DISABLE(huart->hdmarx);
			//HAL_UART_DMAStop(huart);
			HAL_UART_AbortReceive(huart);
			
			UbxUart.RX_flag = 1;
			__HAL_UART_CLEAR_IDLEFLAG(huart);//qingchubiaozhiwei 
			UbxUart.RX_Size = RX_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

			HAL_UART_Receive_DMA(huart, UbxUart.RX_pData,RX_LEN);
		}
	}
	*/
/*
	if(huart->Instance == USART6)
	{		
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET){ 	
			//__HAL_DMA_DISABLE(huart->hdmarx);
			HAL_UART_AbortReceive(huart);
			
			CarUbxUart.RX_flag = 1;
			__HAL_UART_CLEAR_IDLEFLAG(huart);//qingchubiaozhiwei 
			CarUbxUart.RX_Size = RX_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

			HAL_UART_Receive_DMA(huart, CarUbxUart.RX_pData,RX_LEN);
		}
	}
	*/
}


//改变串口3波特率
void USART3_modify_baud_rate(U32 rate)
{
    GPIO_InitTypeDef GPIO_Initure;
    /*发送端口配置*/
    GPIO_Initure.Pin = uart_drv_cfg[1].gpio_tx_pin;
    GPIO_Initure.Mode = uart_drv_cfg[1].gpio_tx_mode;
    GPIO_Initure.Pull = uart_drv_cfg[1].gpio_tx_pull;
    GPIO_Initure.Speed = uart_drv_cfg[1].gpio_tx_speed;
    GPIO_Initure.Alternate = uart_drv_cfg[1].gpio_tx_alternate;
    HAL_GPIO_Init(uart_drv_cfg[1].gpio_tx_port,&GPIO_Initure);

    /*接收端口配置*/
    GPIO_Initure.Pin = uart_drv_cfg[1].gpio_rx_pin;
    GPIO_Initure.Mode = uart_drv_cfg[1].gpio_rx_mode;
    GPIO_Initure.Pull = uart_drv_cfg[1].gpio_rx_pull;
    GPIO_Initure.Speed = uart_drv_cfg[1].gpio_rx_speed;
    GPIO_Initure.Alternate = uart_drv_cfg[1].gpio_rx_alternate;
    HAL_GPIO_Init(uart_drv_cfg[1].gpio_rx_port,&GPIO_Initure);

    /*基本配置*/
    uart_drv_cfg[1].uart_handle->Instance = uart_drv_cfg[1].uart_id;
    uart_drv_cfg[1].uart_handle->Init.BaudRate = rate;
    uart_drv_cfg[1].uart_handle->Init.WordLength = uart_drv_cfg[1].word_len;
    uart_drv_cfg[1].uart_handle->Init.StopBits = uart_drv_cfg[1].stop_bits;
    uart_drv_cfg[1].uart_handle->Init.Parity = uart_drv_cfg[1].parity;
    uart_drv_cfg[1].uart_handle->Init.HwFlowCtl = uart_drv_cfg[1].hw_flow;
    uart_drv_cfg[1].uart_handle->Init.Mode = uart_drv_cfg[1].uart_mode;
    HAL_UART_Init(uart_drv_cfg[1].uart_handle);

    /*中断配置*/
    if(uart_drv_cfg[1].irq_type != UART_INTR_NONE)
    {
        switch(uart_drv_cfg[1].irq_type)
        {
            case UART_INTR_TX:
            {
                __HAL_UART_ENABLE_IT(uart_drv_cfg[1].uart_handle,UART_IT_TXE);	
                break;
            }
            case UART_INTR_RX:
            {
                __HAL_UART_ENABLE_IT(uart_drv_cfg[1].uart_handle,UART_IT_RXNE);	
                break;
            }
            case UART_INTR_TX_RX:
            {
                __HAL_UART_ENABLE_IT(uart_drv_cfg[1].uart_handle,UART_IT_TXE);	
                __HAL_UART_ENABLE_IT(uart_drv_cfg[1].uart_handle,UART_IT_RXNE);	
                break;
            }
            default:
                break;
        }
        
        __HAL_UART_ENABLE_IT(uart_drv_cfg[1].uart_handle, UART_IT_ERR);
        HAL_NVIC_EnableIRQ(uart_drv_cfg[1].irq_num);
        HAL_NVIC_SetPriority(uart_drv_cfg[1].irq_num, uart_drv_cfg[1].prio,uart_drv_cfg[1].sub_prio);	
    }

    /*DMA配置*/
    if(uart_drv_cfg[1].dma_tx_enable == UART_DMA_TX_ENABLE)
    {
        _cfg_uart_dma(uart_drv_cfg[1].dma_tx_handle, uart_drv_cfg[1].uart_handle, uart_drv_cfg[1].dma_tx_stream, uart_drv_cfg[1].dma_tx_request);
    }
}

/*eof*/
