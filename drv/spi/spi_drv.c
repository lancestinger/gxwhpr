/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : spi_drv.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年6月28日
  最近修改   :
  功能描述   : SPI驱动模块
  函数列表   :
  修改历史   :
  1.日    期   : 2019年6月28日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


/*------------------------------头文件------------------------------*/
#include "spi/spi_drv.h"
/*------------------------------头文件------------------------------*/


/*------------------------------???------------------------------*/


/*------------------------------???------------------------------*/


/*------------------------------文件变量------------------------------*/
SPI_HandleTypeDef spi1_handle, spi2_handle, spi3_handle, spi4_handle;

static const spi_drv_t spi_drv_cfg[SPI_NUM] = 
{
	

    /*SPI_1配置项,UWB*/
    {
		&spi1_handle,
		SPI1,
		SPI_MODE_MASTER,
		SPI_DIRECTION_2LINES,
		SPI_DATASIZE_8BIT,
		SPI_NSS_SOFT,
		SPI_BAUDRATEPRESCALER_64,
		SPI_FIRSTBIT_MSB,
		SPI_POLARITY_LOW,
		
		/*nss*/
		GPIOA,
		GPIO_PIN_4,
		GPIO_AF5_SPI1,

		/*clk*/
		GPIOA,
		GPIO_PIN_5,
		GPIO_AF5_SPI1,

		/*miso*/
		GPIOA,
		GPIO_PIN_6,
		GPIO_AF5_SPI1,

		/*mosi*/
		GPIOD,
		GPIO_PIN_7,
		GPIO_AF5_SPI1,
	},	
#if 1
		/*SPI_2配置项,FLASH*/
		{
			&spi2_handle,
			SPI2,
			SPI_MODE_MASTER,
			SPI_DIRECTION_2LINES,
			SPI_DATASIZE_8BIT,
			SPI_NSS_SOFT,
			SPI_BAUDRATEPRESCALER_4,
			SPI_FIRSTBIT_MSB,
			SPI_POLARITY_HIGH,
			
			/*nss*/
			GPIOI,
			GPIO_PIN_0,
			GPIO_AF5_SPI2,
	
			/*clk*/
			GPIOI,
			GPIO_PIN_1,
			GPIO_AF5_SPI2,
	
			/*miso*/
			GPIOI,
			GPIO_PIN_2,
			GPIO_AF5_SPI2,
	
			/*mosi*/
			GPIOI,
			GPIO_PIN_3,
			GPIO_AF5_SPI2,
		},
#endif





    /*SPI_4配置项，ISM330DHCX*/
	{
		&spi4_handle,
		SPI4,
		SPI_MODE_MASTER,
		SPI_DIRECTION_2LINES,
		SPI_DATASIZE_8BIT,
		SPI_NSS_SOFT,
		SPI_BAUDRATEPRESCALER_128,
		SPI_FIRSTBIT_MSB,
		SPI_POLARITY_HIGH,
		
		/*nss*/
		GPIOE,
		GPIO_PIN_4,
		GPIO_AF5_SPI4,

		/*clk*/
		GPIOE,
		GPIO_PIN_2,
		GPIO_AF5_SPI4,

		/*miso*/
		GPIOE,
		GPIO_PIN_5,
		GPIO_AF5_SPI4,

		/*mosi*/
		GPIOE,
		GPIO_PIN_6,
		GPIO_AF5_SPI4,
	},
};
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/

/*****************************************************************************
 函 数 名  : HAL_SPI_MspInit
 功能描述  : SPI初始化回调函数
 输入参数  : SPI_HandleTypeDef *hspi  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月4日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI1)
	{
		__HAL_RCC_SPI1_CLK_ENABLE(); 
	}
	else if(hspi->Instance == SPI2)
	{
		__HAL_RCC_SPI2_CLK_ENABLE(); 
	}
	else if(hspi->Instance == SPI3)
	{
		__HAL_RCC_SPI3_CLK_ENABLE(); 
	}
    else if(hspi->Instance == SPI4)
	{
		__HAL_RCC_SPI4_CLK_ENABLE(); 
	}
}
/*****************************************************************************
 函 数 名  : spi_drv_init
 功能描述  : SPI模块初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月28日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void spi_drv_init(void)
{
	U8 i = 0;
	GPIO_InitTypeDef gpio_initure;

	GLOBAL_MEMSET(&spi1_handle, 0, sizeof(spi1_handle));
	GLOBAL_MEMSET(&spi2_handle, 0, sizeof(spi2_handle));
	//GLOBAL_MEMSET(&spi3_handle, 0, sizeof(spi3_handle));
    GLOBAL_MEMSET(&spi4_handle, 0, sizeof(spi4_handle));
	
	for(i = 0; i < SPI_NUM; i++)
	{
		GLOBAL_MEMSET(&gpio_initure, 0, sizeof(gpio_initure));
		spi_drv_cfg[i].p_spi_handle->Instance = spi_drv_cfg[i].spi_sel;
		spi_drv_cfg[i].p_spi_handle->Init.Mode = spi_drv_cfg[i].mode;
		spi_drv_cfg[i].p_spi_handle->Init.Direction = spi_drv_cfg[i].direction;
		spi_drv_cfg[i].p_spi_handle->Init.DataSize = spi_drv_cfg[i].data_size;
		spi_drv_cfg[i].p_spi_handle->Init.CLKPolarity = spi_drv_cfg[i].clk_polarity;
		spi_drv_cfg[i].p_spi_handle->Init.CLKPhase = SPI_PHASE_2EDGE;
		if(i==SPI_1) spi_drv_cfg[i].p_spi_handle->Init.CLKPhase = SPI_PHASE_1EDGE;
			
		spi_drv_cfg[i].p_spi_handle->Init.NSS = spi_drv_cfg[i].nss_sel;
		spi_drv_cfg[i].p_spi_handle->Init.NSSPMode=SPI_NSS_PULSE_DISABLE;//NSS信号脉冲失能
		//spi_drv_cfg[i].p_spi_handle->Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;  //SPI主模式IO状态保持使能
		if(i!=SPI_1) spi_drv_cfg[i].p_spi_handle->Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;
		spi_drv_cfg[i].p_spi_handle->Init.BaudRatePrescaler = spi_drv_cfg[i].baud_rate_prescale;
		spi_drv_cfg[i].p_spi_handle->Init.FirstBit = spi_drv_cfg[i].first_bit;
		spi_drv_cfg[i].p_spi_handle->Init.TIMode = SPI_TIMODE_DISABLE;
		spi_drv_cfg[i].p_spi_handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		spi_drv_cfg[i].p_spi_handle->Init.CRCPolynomial = 0x0;
		if(i!=SPI_4) spi_drv_cfg[i].p_spi_handle->Init.CRCPolynomial = 0x7;
		//if(i==SPI_1) spi_drv_cfg[i].p_spi_handle->Init.CRCPolynomial = 7;

		if(spi_drv_cfg[i].nss_sel == SPI_NSS_SOFT)
		{
			gpio_initure.Pin = spi_drv_cfg[i].nss_gpio_pin;
			gpio_initure.Mode = GPIO_MODE_OUTPUT_PP;
			gpio_initure.Pull = GPIO_PULLUP;//GPIO_PULLUP
			if(i==SPI_4)
				gpio_initure.Pull = GPIO_NOPULL;//GPIO_PULLUP
			gpio_initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			HAL_GPIO_Init(spi_drv_cfg[i].nss_gpio_port, &gpio_initure);
			HAL_GPIO_WritePin(spi_drv_cfg[i].nss_gpio_port, spi_drv_cfg[i].nss_gpio_pin, GPIO_PIN_SET);
		}
		else
		{
			gpio_initure.Pin = spi_drv_cfg[i].nss_gpio_pin;
			gpio_initure.Alternate = spi_drv_cfg[i].nss_gpio_alternate;
			gpio_initure.Pull = GPIO_PULLUP;
			gpio_initure.Mode = GPIO_MODE_AF_PP;
			gpio_initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			HAL_GPIO_Init(spi_drv_cfg[i].nss_gpio_port, &gpio_initure);
		}

		gpio_initure.Mode = GPIO_MODE_AF_PP;
		gpio_initure.Pull = GPIO_NOPULL;
		gpio_initure.Pin = spi_drv_cfg[i].clk_gpio_pin;
		gpio_initure.Alternate = spi_drv_cfg[i].clk_gpio_alternate;
		gpio_initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(spi_drv_cfg[i].clk_gpio_port, &gpio_initure);

		gpio_initure.Pin = spi_drv_cfg[i].miso_gpio_pin;
		gpio_initure.Alternate = spi_drv_cfg[i].miso_gpio_alternate;
		gpio_initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(spi_drv_cfg[i].miso_gpio_port, &gpio_initure);

		gpio_initure.Pin = spi_drv_cfg[i].mosi_gpio_pin;
		gpio_initure.Alternate = spi_drv_cfg[i].mosi_gpio_alternate;
		gpio_initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(spi_drv_cfg[i].mosi_gpio_port, &gpio_initure);
		
		HAL_SPI_Init(spi_drv_cfg[i].p_spi_handle);//初始化

		if(i!=SPI_4){
	    	__HAL_SPI_ENABLE(spi_drv_cfg[i].p_spi_handle);  //使能SPI
	    	spi_read_write_byte((spi_type_enum)i, 0xFF);
		}
	}
	NOTE_PRINT(("SPI初始化完成!!!!"));
	delay_ms(20);

}

//void spi_nss_soft_set()

/*****************************************************************************
 函 数 名  : spi_read_write_byte
 功能描述  : 读写1个字节
 输入参数  : spi_type_enum type  
             U8 data             
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月28日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
U8 spi_read_write_byte(spi_type_enum type, U8 data)
{
	U8 rx_data = 0;

	if(type >= SPI_NUM) return 0;
	HAL_SPI_TransmitReceive(spi_drv_cfg[type].p_spi_handle, &data, &rx_data,1, 1000);   
	return rx_data;
}

/*****************************************************************************
 函 数 名  : spi_write_buffer
 功能描述  : SPI写缓存
 输入参数  : spi_type_enum type  
             U8* buf             
             U32 len             
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月28日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void spi_write_buffer(spi_type_enum type, IN U8* buf, IN U32 len)
{
	if(type >= SPI_NUM) return;
	HAL_SPI_Transmit(spi_drv_cfg[type].p_spi_handle, buf, len, 1000);
}

/*****************************************************************************
 函 数 名  : spi_read_buffer
 功能描述  : SPI读buffer
 输入参数  : spi_type_enum type  
             INOUT U8* buf       
             IN U32 len          
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月28日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void spi_read_buffer(spi_type_enum type, INOUT U8* buf, IN U32 len)
{
	if(type >= SPI_NUM) return;
	HAL_SPI_Receive(spi_drv_cfg[type].p_spi_handle, buf, len, 1000);
}

/*****************************************************************************
 函 数 名  : spi_set_datasize
 功能描述  : SPI设置帧数据宽度
 输入参数  : spi_type_enum type  
             U8 size             
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月22日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void spi_set_datasize(spi_type_enum type, U8 size)
{
#if 0
	U32 temp32 = 0;
	
	if(type >= SPI_NUM) return;
	__HAL_SPI_DISABLE(spi_drv_cfg[type].p_spi_handle);            //关闭SPI
	temp32 = spi_drv_cfg[type].p_spi_handle->Instance->CFG1;
	spi_drv_cfg[type].p_spi_handle->Instance->CFG1 = ((temp32 & 0xFFFFFFE0) | (size & 0x1F));
	__HAL_SPI_ENABLE(spi_drv_cfg[type].p_spi_handle);            //关闭SPI
#endif	
}

/*****************************************************************************
 函 数 名  : spi_set_clkphase
 功能描述  : 设置时钟相位
 输入参数  : spi_type_enum type  
             U32 clkphase        
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月5日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void spi_set_clkphase(spi_type_enum type, U32 clkphase)
{
#if 0
	U32 temp32 = 0;
	
	if(type >= SPI_NUM) return;
	__HAL_SPI_DISABLE(spi_drv_cfg[type].p_spi_handle);            //关闭SPI
	temp32 = spi_drv_cfg[type].p_spi_handle->Instance->CFG2;
	spi_drv_cfg[type].p_spi_handle->Instance->CFG2 = ((temp32 & 0xFEFFFFFF) | (clkphase));
	__HAL_SPI_ENABLE(spi_drv_cfg[type].p_spi_handle);            //关闭SPI
#endif	
}

/*****************************************************************************
 函 数 名  : spi_set_baudrate
 功能描述  : 设置通信速率
 输入参数  : spi_type_enum type  
            U32 prescale         
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月24日
    作    者   : sunj
    修改内容   : 新生成函数

*****************************************************************************/
void spi_set_baudrate(spi_type_enum type, U32 prescale)
{
    U32 temp32 = 0;
	U8 ret = 0;

    if(type >= SPI_NUM) return;
    __HAL_SPI_DISABLE(spi_drv_cfg[type].p_spi_handle);            //关闭SPI
    spi_drv_cfg[type].p_spi_handle->Init.BaudRatePrescaler = prescale;
	HAL_SPI_Init(spi_drv_cfg[type].p_spi_handle);//初始化		
	delay_ms(5);
    __HAL_SPI_ENABLE(spi_drv_cfg[type].p_spi_handle);            //使能SPI   
	//ret = spi_read_write_byte((spi_type_enum)type, 0xFF);
	//GLOBAL_PRINT(("spi_read_write_byte RET = %d\r\n",ret));
}




/*****************************************************************************
 函 数 名  : spi_fast_write_buffer
 功能描述  : SPI快速写缓存
 输入参数  : spi_type_enum type  
             U8* buf             
             U32 len             
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年1月6日
    作    者   : wx
    修改内容   : 新生成函数

*****************************************************************************/
void spi_fast_write_buffer(spi_type_enum type, IN U8* buf, IN U32 len)
{
	U32 i;

	if(type >= SPI_NUM) return;

	for(i=0; i<len; i++)
	{
			spi_drv_cfg[type].spi_sel->TXDR = buf[i];
			while((spi_drv_cfg[type].spi_sel->SR & SPI_FLAG_RXNE) == (uint16_t)RESET); 
			spi_drv_cfg[type].spi_sel->TXDR;
	} 	
	
}


/*****************************************************************************
 函 数 名  : spi_fast_read_buffer
 功能描述  : SPI读buffer
 输入参数  : spi_type_enum type  
             INOUT U8* buf       
             IN U32 len          
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年1月6日
    作    者   : wx
    修改内容   : 新生成函数

*****************************************************************************/
void spi_fast_read_buffer(spi_type_enum type, INOUT U8* buf, IN U32 len)
{
	U32 i;

	if(type >= SPI_NUM) return;
	
	for(i=0; i<len; i++)
	{
			spi_drv_cfg[type].spi_sel->RXDR = 0;
			while((spi_drv_cfg[type].spi_sel->SR & SPI_FLAG_RXNE) == (uint16_t)RESET); 
			buf[i] = spi_drv_cfg[type].spi_sel->RXDR;
	} 	

}


void spi_set_enable(spi_type_enum type)
{

	__HAL_SPI_ENABLE(spi_drv_cfg[type].p_spi_handle);

}



void spi_set_disable(spi_type_enum type)
{
	//SPI_CloseTransfer(spi_drv_cfg[type].p_spi_handle);
	__HAL_SPI_CLEAR_EOTFLAG(spi_drv_cfg[type].p_spi_handle);
	__HAL_SPI_CLEAR_TXTFFLAG(spi_drv_cfg[type].p_spi_handle);

	/* Disable SPI peripheral */
	__HAL_SPI_DISABLE(spi_drv_cfg[type].p_spi_handle);

	/* Disable ITs */
	__HAL_SPI_DISABLE_IT(spi_drv_cfg[type].p_spi_handle, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));

	/* Disable Tx DMA Request */
	CLEAR_BIT(spi_drv_cfg[type].p_spi_handle->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

	//__HAL_SPI_DISABLE(spi_drv_cfg[type].p_spi_handle);

}



/*eof*/

