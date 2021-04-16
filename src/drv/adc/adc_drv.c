/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : adc_drv.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年8月23日
  最近修改   :
  功能描述   : 内部ADC驱动程序
  函数列表   :
  修改历史   :
  1.日    期   : 2019年8月23日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


/*------------------------------头文件------------------------------*/
#include "adc/adc_drv.h"
/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/

/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
static ADC_HandleTypeDef adc1_handle;
static ADC_HandleTypeDef adc3_handle;
static const adc_cfg_t adc_drv_cfg[ADC_NUM] = 
{
	/*ADC_1*/
	{
		&adc3_handle,
		ADC3,
		ADC_CHANNEL_TEMPSENSOR,
		ADC_CLOCK_SYNC_PCLK_DIV4,
		ADC_RESOLUTION_16B,
		FALSE,
		0,
		0,
	},

		/*ADC_2*/
	{
		&adc1_handle,
		ADC1,
		ADC_CHANNEL_10,
		ADC_CLOCK_SYNC_PCLK_DIV4,
		ADC_RESOLUTION_16B,
		TRUE,
		GPIOC,
		GPIO_PIN_0,
	},

		/*ADC_3*/
	{
		&adc1_handle,
		ADC1,
		ADC_CHANNEL_12,
		ADC_CLOCK_SYNC_PCLK_DIV4,
		ADC_RESOLUTION_16B,
		TRUE,
		GPIOC,
		GPIO_PIN_2,
	},
};
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	RCC_PeriphCLKInitTypeDef  ADCClkInitStruct;

	ADCClkInitStruct.PeriphClockSelection=RCC_PERIPHCLK_ADC; 
	ADCClkInitStruct.AdcClockSelection=RCC_ADCCLKSOURCE_CLKP; 
	HAL_RCCEx_PeriphCLKConfig(&ADCClkInitStruct);
	if((hadc->Instance == ADC1) || (hadc->Instance == ADC2))
	{
		__HAL_RCC_ADC12_CLK_ENABLE();            //使能ADC12时钟

	}
	else if(hadc->Instance == ADC3)
	{
		__HAL_RCC_ADC3_CLK_ENABLE();            //使能ADC3时钟

	}
}

//获得ADC值
//ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
//返回值:转换结果
U16 get_adc_value(adc_chn_enum adc_ch)   
{
	ADC_HandleTypeDef* hadc;
    ADC_ChannelConfTypeDef adc_channel_conf;

	GLOBAL_MEMSET(&adc_channel_conf, 0, sizeof(adc_channel_conf));
	hadc = adc_drv_cfg[adc_ch].adc_handle;
    adc_channel_conf.Channel = adc_drv_cfg[adc_ch].chn;                                   //通道
    adc_channel_conf.Rank=ADC_REGULAR_RANK_1;                  	// 1个序列
    adc_channel_conf.SamplingTime=ADC_SAMPLETIME_810CYCLES_5;     	//采样时间       
	adc_channel_conf.SingleDiff=ADC_SINGLE_ENDED;  				//单边采集          		
	adc_channel_conf.OffsetNumber=ADC_OFFSET_NONE;             	
	adc_channel_conf.Offset=0;   
    HAL_ADC_ConfigChannel(hadc, &adc_channel_conf);        //通道配置

    HAL_ADC_Start(hadc);                               		//开启ADC
	
    HAL_ADC_PollForConversion(hadc, 50);                		//轮询转换
	return (U16)HAL_ADC_GetValue(hadc);	            		//返回最近一次ADC规则组的转换结果
}

/*****************************************************************************
 函 数 名  : adc_drv_init
 功能描述  : ADC驱动初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月23日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void adc_drv_init(void)
{
	U8 i = 0;

	for(i = ADC_1; i < ADC_NUM; i++)
	{
		GLOBAL_MEMSET(adc_drv_cfg[i].adc_handle, 0, sizeof(ADC_HandleTypeDef));
		adc_drv_cfg[i].adc_handle->Instance = adc_drv_cfg[i].adc_sel;
		adc_drv_cfg[i].adc_handle->Init.ClockPrescaler = adc_drv_cfg[i].clk_precaler;
		adc_drv_cfg[i].adc_handle->Init.Resolution = adc_drv_cfg[i].adc_resolution;
		adc_drv_cfg[i].adc_handle->Init.ScanConvMode = DISABLE;                    	//非扫描模式
		adc_drv_cfg[i].adc_handle->Init.EOCSelection = ADC_EOC_SINGLE_CONV;       	//关闭EOC中断
		adc_drv_cfg[i].adc_handle->Init.LowPowerAutoWait = DISABLE;				//自动低功耗关闭	
		adc_drv_cfg[i].adc_handle->Init.ContinuousConvMode = DISABLE;               //关闭连续转换
		adc_drv_cfg[i].adc_handle->Init.NbrOfConversion = 1;                        // 1个转换在规则序列中 也就是只转换规则序列1 
		adc_drv_cfg[i].adc_handle->Init.DiscontinuousConvMode = DISABLE;            //禁止不连续采样模式
		adc_drv_cfg[i].adc_handle->Init.NbrOfDiscConversion = 0;                    //不连续采样通道数为0
		adc_drv_cfg[i].adc_handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;      //软件触发
		adc_drv_cfg[i].adc_handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;//使用软件触发
		//		adc_drv_cfg[i].adc_handle->Init.BoostMode = ENABLE;							//BOOT模式关闭	modified by sunj 2019-11-12 18:20
		adc_drv_cfg[i].adc_handle->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;			//有新的数据的死后直接覆盖掉旧数据
		adc_drv_cfg[i].adc_handle->Init.OversamplingMode = DISABLE;					//过采样关闭
		adc_drv_cfg[i].adc_handle->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;  //规则通道的数据仅仅保存在DR寄存器里面
		HAL_ADC_Init(adc_drv_cfg[i].adc_handle);                                 //初始化 
		if(adc_drv_cfg[i].gpio_enable)
		{
			GPIO_InitTypeDef gpio_initure;

			GLOBAL_MEMSET(&gpio_initure, 0, sizeof(gpio_initure));
			gpio_initure.Pin = adc_drv_cfg[i].gpio_pin;            //PA0 PC2
		    gpio_initure.Mode = GPIO_MODE_ANALOG;     //模拟
		    gpio_initure.Pull = GPIO_NOPULL;          //不带上下拉
		    HAL_GPIO_Init(adc_drv_cfg[i].gpio_port, &gpio_initure);
		}
		HAL_ADCEx_Calibration_Start(adc_drv_cfg[i].adc_handle, ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); //ADC校准
	}
	NOTE_PRINT(("ADC 初始化完成!!!!!\r\n"));
}


/*****************************************************************************
 函 数 名  : adc_get_temperature
 功能描述  : 获取ADC的温度传感器的值
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月23日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
F32 adc_get_temperature(void)
{
	F32 ret = 0, temp = 0;
	U16 ts_cal1,ts_cal2;
	S32 adc_val = 0;
    
    ts_cal1 = *(U16*)(0X1FF1E820);
    ts_cal2 = *(U16*)(0X1FF1E840);	
	temp = (110.0f - 30.0f)/(ts_cal2-ts_cal1);
	adc_val = get_adc_value(ADC_TEMPERATURE);
	ret = temp*(adc_val-ts_cal1)+30.0f;
	//GLOBAL_TRACE(("ts_cal1 = 0x%04X, ts_cal2 = 0x%04X, adc_val = %u\r\n", ts_cal1, ts_cal2, adc_val));
	return ret;
}

/*eof*/

