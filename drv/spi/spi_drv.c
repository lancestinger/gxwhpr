/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : spi_drv.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��6��28��
  ����޸�   :
  ��������   : SPI����ģ��
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��6��28��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


/*------------------------------ͷ�ļ�------------------------------*/
#include "spi/spi_drv.h"
/*------------------------------ͷ�ļ�------------------------------*/


/*------------------------------???------------------------------*/


/*------------------------------???------------------------------*/


/*------------------------------�ļ�����------------------------------*/
SPI_HandleTypeDef spi1_handle, spi2_handle, spi3_handle, spi4_handle;

static const spi_drv_t spi_drv_cfg[SPI_NUM] = 
{
	

    /*SPI_1������,UWB*/
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
		/*SPI_2������,FLASH*/
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





    /*SPI_4�����ISM330DHCX*/
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
/*------------------------------�ļ�����------------------------------*/


/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/

/*****************************************************************************
 �� �� ��  : HAL_SPI_MspInit
 ��������  : SPI��ʼ���ص�����
 �������  : SPI_HandleTypeDef *hspi  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��8��4��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
 �� �� ��  : spi_drv_init
 ��������  : SPIģ���ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��28��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
		spi_drv_cfg[i].p_spi_handle->Init.NSSPMode=SPI_NSS_PULSE_DISABLE;//NSS�ź�����ʧ��
		//spi_drv_cfg[i].p_spi_handle->Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;  //SPI��ģʽIO״̬����ʹ��
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
		
		HAL_SPI_Init(spi_drv_cfg[i].p_spi_handle);//��ʼ��

		if(i!=SPI_4){
	    	__HAL_SPI_ENABLE(spi_drv_cfg[i].p_spi_handle);  //ʹ��SPI
	    	spi_read_write_byte((spi_type_enum)i, 0xFF);
		}
	}
	NOTE_PRINT(("SPI��ʼ�����!!!!"));
	delay_ms(20);

}

//void spi_nss_soft_set()

/*****************************************************************************
 �� �� ��  : spi_read_write_byte
 ��������  : ��д1���ֽ�
 �������  : spi_type_enum type  
             U8 data             
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��28��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
U8 spi_read_write_byte(spi_type_enum type, U8 data)
{
	U8 rx_data = 0;

	if(type >= SPI_NUM) return 0;
	HAL_SPI_TransmitReceive(spi_drv_cfg[type].p_spi_handle, &data, &rx_data,1, 1000);   
	return rx_data;
}

/*****************************************************************************
 �� �� ��  : spi_write_buffer
 ��������  : SPIд����
 �������  : spi_type_enum type  
             U8* buf             
             U32 len             
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��28��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void spi_write_buffer(spi_type_enum type, IN U8* buf, IN U32 len)
{
	if(type >= SPI_NUM) return;
	HAL_SPI_Transmit(spi_drv_cfg[type].p_spi_handle, buf, len, 1000);
}

/*****************************************************************************
 �� �� ��  : spi_read_buffer
 ��������  : SPI��buffer
 �������  : spi_type_enum type  
             INOUT U8* buf       
             IN U32 len          
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��28��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void spi_read_buffer(spi_type_enum type, INOUT U8* buf, IN U32 len)
{
	if(type >= SPI_NUM) return;
	HAL_SPI_Receive(spi_drv_cfg[type].p_spi_handle, buf, len, 1000);
}

/*****************************************************************************
 �� �� ��  : spi_set_datasize
 ��������  : SPI����֡���ݿ��
 �������  : spi_type_enum type  
             U8 size             
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��22��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void spi_set_datasize(spi_type_enum type, U8 size)
{
#if 0
	U32 temp32 = 0;
	
	if(type >= SPI_NUM) return;
	__HAL_SPI_DISABLE(spi_drv_cfg[type].p_spi_handle);            //�ر�SPI
	temp32 = spi_drv_cfg[type].p_spi_handle->Instance->CFG1;
	spi_drv_cfg[type].p_spi_handle->Instance->CFG1 = ((temp32 & 0xFFFFFFE0) | (size & 0x1F));
	__HAL_SPI_ENABLE(spi_drv_cfg[type].p_spi_handle);            //�ر�SPI
#endif	
}

/*****************************************************************************
 �� �� ��  : spi_set_clkphase
 ��������  : ����ʱ����λ
 �������  : spi_type_enum type  
             U32 clkphase        
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��8��5��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void spi_set_clkphase(spi_type_enum type, U32 clkphase)
{
#if 0
	U32 temp32 = 0;
	
	if(type >= SPI_NUM) return;
	__HAL_SPI_DISABLE(spi_drv_cfg[type].p_spi_handle);            //�ر�SPI
	temp32 = spi_drv_cfg[type].p_spi_handle->Instance->CFG2;
	spi_drv_cfg[type].p_spi_handle->Instance->CFG2 = ((temp32 & 0xFEFFFFFF) | (clkphase));
	__HAL_SPI_ENABLE(spi_drv_cfg[type].p_spi_handle);            //�ر�SPI
#endif	
}

/*****************************************************************************
 �� �� ��  : spi_set_baudrate
 ��������  : ����ͨ������
 �������  : spi_type_enum type  
            U32 prescale         
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��12��24��
    ��    ��   : sunj
    �޸�����   : �����ɺ���

*****************************************************************************/
void spi_set_baudrate(spi_type_enum type, U32 prescale)
{
    U32 temp32 = 0;
	U8 ret = 0;

    if(type >= SPI_NUM) return;
    __HAL_SPI_DISABLE(spi_drv_cfg[type].p_spi_handle);            //�ر�SPI
    spi_drv_cfg[type].p_spi_handle->Init.BaudRatePrescaler = prescale;
	HAL_SPI_Init(spi_drv_cfg[type].p_spi_handle);//��ʼ��		
	delay_ms(5);
    __HAL_SPI_ENABLE(spi_drv_cfg[type].p_spi_handle);            //ʹ��SPI   
	//ret = spi_read_write_byte((spi_type_enum)type, 0xFF);
	//GLOBAL_PRINT(("spi_read_write_byte RET = %d\r\n",ret));
}




/*****************************************************************************
 �� �� ��  : spi_fast_write_buffer
 ��������  : SPI����д����
 �������  : spi_type_enum type  
             U8* buf             
             U32 len             
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��1��6��
    ��    ��   : wx
    �޸�����   : �����ɺ���

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
 �� �� ��  : spi_fast_read_buffer
 ��������  : SPI��buffer
 �������  : spi_type_enum type  
             INOUT U8* buf       
             IN U32 len          
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��1��6��
    ��    ��   : wx
    �޸�����   : �����ɺ���

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

