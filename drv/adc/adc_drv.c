/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : adc_drv.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��8��23��
  ����޸�   :
  ��������   : �ڲ�ADC��������
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��8��23��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


/*------------------------------ͷ�ļ�------------------------------*/
#include "adc/adc_drv.h"
/*------------------------------ͷ�ļ�------------------------------*/


/*------------------------------�ļ���------------------------------*/

/*------------------------------�ļ���------------------------------*/


/*------------------------------�ļ�����------------------------------*/
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
/*------------------------------�ļ�����------------------------------*/


/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	RCC_PeriphCLKInitTypeDef  ADCClkInitStruct;

	ADCClkInitStruct.PeriphClockSelection=RCC_PERIPHCLK_ADC; 
	ADCClkInitStruct.AdcClockSelection=RCC_ADCCLKSOURCE_CLKP; 
	HAL_RCCEx_PeriphCLKConfig(&ADCClkInitStruct);
	if((hadc->Instance == ADC1) || (hadc->Instance == ADC2))
	{
		__HAL_RCC_ADC12_CLK_ENABLE();            //ʹ��ADC12ʱ��

	}
	else if(hadc->Instance == ADC3)
	{
		__HAL_RCC_ADC3_CLK_ENABLE();            //ʹ��ADC3ʱ��

	}
}

//���ADCֵ
//ch: ͨ��ֵ 0~16��ȡֵ��ΧΪ��ADC_CHANNEL_0~ADC_CHANNEL_16
//����ֵ:ת�����
U16 get_adc_value(adc_chn_enum adc_ch)   
{
	ADC_HandleTypeDef* hadc;
    ADC_ChannelConfTypeDef adc_channel_conf;

	GLOBAL_MEMSET(&adc_channel_conf, 0, sizeof(adc_channel_conf));
	hadc = adc_drv_cfg[adc_ch].adc_handle;
    adc_channel_conf.Channel = adc_drv_cfg[adc_ch].chn;                                   //ͨ��
    adc_channel_conf.Rank=ADC_REGULAR_RANK_1;                  	// 1������
    adc_channel_conf.SamplingTime=ADC_SAMPLETIME_810CYCLES_5;     	//����ʱ��       
	adc_channel_conf.SingleDiff=ADC_SINGLE_ENDED;  				//���߲ɼ�          		
	adc_channel_conf.OffsetNumber=ADC_OFFSET_NONE;             	
	adc_channel_conf.Offset=0;   
    HAL_ADC_ConfigChannel(hadc, &adc_channel_conf);        //ͨ������

    HAL_ADC_Start(hadc);                               		//����ADC
	
    HAL_ADC_PollForConversion(hadc, 50);                		//��ѯת��
	return (U16)HAL_ADC_GetValue(hadc);	            		//�������һ��ADC�������ת�����
}

/*****************************************************************************
 �� �� ��  : adc_drv_init
 ��������  : ADC������ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��8��23��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
		adc_drv_cfg[i].adc_handle->Init.ScanConvMode = DISABLE;                    	//��ɨ��ģʽ
		adc_drv_cfg[i].adc_handle->Init.EOCSelection = ADC_EOC_SINGLE_CONV;       	//�ر�EOC�ж�
		adc_drv_cfg[i].adc_handle->Init.LowPowerAutoWait = DISABLE;				//�Զ��͹��Ĺر�	
		adc_drv_cfg[i].adc_handle->Init.ContinuousConvMode = DISABLE;               //�ر�����ת��
		adc_drv_cfg[i].adc_handle->Init.NbrOfConversion = 1;                        // 1��ת���ڹ��������� Ҳ����ֻת����������1 
		adc_drv_cfg[i].adc_handle->Init.DiscontinuousConvMode = DISABLE;            //��ֹ����������ģʽ
		adc_drv_cfg[i].adc_handle->Init.NbrOfDiscConversion = 0;                    //����������ͨ����Ϊ0
		adc_drv_cfg[i].adc_handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;      //�������
		adc_drv_cfg[i].adc_handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;//ʹ���������
		//		adc_drv_cfg[i].adc_handle->Init.BoostMode = ENABLE;							//BOOTģʽ�ر�	modified by sunj 2019-11-12 18:20
		adc_drv_cfg[i].adc_handle->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;			//���µ����ݵ�����ֱ�Ӹ��ǵ�������
		adc_drv_cfg[i].adc_handle->Init.OversamplingMode = DISABLE;					//�������ر�
		adc_drv_cfg[i].adc_handle->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;  //����ͨ�������ݽ���������DR�Ĵ�������
		HAL_ADC_Init(adc_drv_cfg[i].adc_handle);                                 //��ʼ�� 
		if(adc_drv_cfg[i].gpio_enable)
		{
			GPIO_InitTypeDef gpio_initure;

			GLOBAL_MEMSET(&gpio_initure, 0, sizeof(gpio_initure));
			gpio_initure.Pin = adc_drv_cfg[i].gpio_pin;            //PA0 PC2
		    gpio_initure.Mode = GPIO_MODE_ANALOG;     //ģ��
		    gpio_initure.Pull = GPIO_NOPULL;          //����������
		    HAL_GPIO_Init(adc_drv_cfg[i].gpio_port, &gpio_initure);
		}
		HAL_ADCEx_Calibration_Start(adc_drv_cfg[i].adc_handle, ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); //ADCУ׼
	}
	NOTE_PRINT(("ADC ��ʼ�����!!!!!\r\n"));
}


/*****************************************************************************
 �� �� ��  : adc_get_temperature
 ��������  : ��ȡADC���¶ȴ�������ֵ
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��8��23��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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

