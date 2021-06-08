/******************************************************************************

                  版权所有 (C), 2015-2020, Winlab

 ******************************************************************************
  文 件 名   : gpio_drv.c
  版 本 号   : 初稿
  作    者   : Winlab
  生成日期   : 2019年6月22日
  最近修改   :
  功能描述   : GPIO模块
  函数列表   :
  修改历史   :
  1.日    期   : 2019年6月22日
    作    者   : Winlab
    修改内容   : 创建文件

******************************************************************************/

/*------------------------------头文件------------------------------*/
#include "gpio/gpio_drv.h"
#include "project_def.h"
#include "apl/imu/Imu.h"
#include "fml/gnss/nmea.h"
#include "uwb_post/uwb_post.h"
/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/

/*------------------------------文件宏------------------------------*/

osEventFlagsId_t evt_id; // message queue id
/*------------------------------文件变量------------------------------*/
static const gpio_drv_t gpio_drv_cfg[GPIO_NUM] = 
{
	/*GPIO 1,PHY_RESET*/
	{
		GPIOC,
		GPIO_PIN_3,
		GPIO_MODE_OUTPUT_PP,

		FALSE,
		NonMaskableInt_IRQn,
		0,0,
	},
	/*GPIO 2,ISM330_SS_PIN*/
	{
		GPIOE,
		GPIO_PIN_4,
		GPIO_MODE_OUTPUT_PP,

		FALSE,
		NonMaskableInt_IRQn,
		0,0,
	},
	/*GPIO 3,ZED_RST_PIN*/
	{
		GPIOI,
		GPIO_PIN_4,
		GPIO_MODE_OUTPUT_PP,

		FALSE,
		NonMaskableInt_IRQn,
		0,0,
	},
	/*GPIO 4,PANT_PIN*/
	{
		GPIOF,
		GPIO_PIN_2,
		GPIO_MODE_OUTPUT_PP,

		FALSE,
		NonMaskableInt_IRQn,
		0,0,
	},
	/*GPIO 5,UBX_1PPS_PIN*/
	{	
		GPIOA,
		GPIO_PIN_0,
		GPIO_MODE_IT_RISING,

		TRUE,
		EXTI0_IRQn,
		3,0,
	},
	/*GPIO 6,FLASH_SS_PIN*/
	{
		GPIOI,
		GPIO_PIN_0,
		GPIO_MODE_OUTPUT_PP,

		FALSE,
		NonMaskableInt_IRQn,
		0,0,
	},
    /*GPIO 7,SIM_RST|SIM_PWR*/
    {
        GPIOB,
        GPIO_PIN_3|GPIO_PIN_4,
        GPIO_MODE_OUTPUT_PP,

        FALSE,
		NonMaskableInt_IRQn,
        0,0,
    },	
    /*GPIO 8,SIM_DEF|SIM_STATUS*/
    {
        GPIOB,
        GPIO_PIN_5|GPIO_PIN_6,
        GPIO_MODE_INPUT,

		FALSE,
		NonMaskableInt_IRQn,
        0,0,
    },
	/*GPIO 9,ISM330DHXC INT1*/
    {
        GPIOG,
        GPIO_PIN_13,
        GPIO_MODE_IT_RISING,

		TRUE,
		EXTI15_10_IRQn,
        5,0,
    },
	/*GPIO 10,ISM330DHXC INT2*/
    {
        GPIOG,
        GPIO_PIN_12,
        GPIO_MODE_IT_RISING,

		TRUE,
		EXTI15_10_IRQn,
        6,1,
    },
	/*GPIO 11,Iis2mdc INT/DRDY*/
    {
        GPIOG,
        GPIO_PIN_14,
        GPIO_MODE_IT_RISING,

		TRUE,
		EXTI15_10_IRQn,
        7,2,
    },  
    /*GPIO 12,UWB IRQ*/
    {
        GPIOG,
        GPIO_PIN_5,
        GPIO_MODE_IT_RISING,

		TRUE,
		EXTI9_5_IRQn,
		1,1,
    },
	//GPIO13 PA15,LED//
    {
        GPIOA,
        GPIO_PIN_15,
        GPIO_MODE_OUTPUT_PP,

        FALSE,
        NonMaskableInt_IRQn,
        0,0,
    },	
    //GPIO14 PC10,LED//
    {
        GPIOC,
        GPIO_PIN_10,
        GPIO_MODE_OUTPUT_PP,

        FALSE,
        NonMaskableInt_IRQn,
        0,0,
    },
    //GPIO15 PC11,LED//
    {
        GPIOC,
        GPIO_PIN_11,
        GPIO_MODE_OUTPUT_PP,

        FALSE,
        NonMaskableInt_IRQn,
        0,0,
    },
    //GPIO16 PC12,LED//
    {
        GPIOC,
        GPIO_PIN_12,
        GPIO_MODE_OUTPUT_PP,

        FALSE,
        NonMaskableInt_IRQn,
        0,0,
    },
    //GPIO17 PD3,LED//
    {
        GPIOD,
        GPIO_PIN_3,
        GPIO_MODE_OUTPUT_PP,

        FALSE,
        NonMaskableInt_IRQn,
        0,0,
    },
    //GPIO18 PD0,LED//
    {
        GPIOD,
        GPIO_PIN_0,
        GPIO_MODE_OUTPUT_PP,

        FALSE,
        NonMaskableInt_IRQn,
        0,0,
    },

	
};
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/


/*****************************************************************************
 函 数 名  : gpio_drv_set
 功能描述  : GPIO设置
 输入参数  : gpio_type_num gpio_type  
             U8 val                   
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月22日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
void gpio_drv_set(gpio_type_num gpio_type, U8 val)
{
	if(gpio_type >= GPIO_NUM)
	{
		ERR_PRINT(("GPIO参数非法!!!!\r\n"));
		return;
	}

	if(gpio_drv_cfg[gpio_type].irq_enable == TRUE)
	{
		ERR_PRINT(("该GPIO为中断输入,禁止写入!!!!!\r\n"));
		return;
	}
	if(val)
	{
		HAL_GPIO_WritePin(gpio_drv_cfg[gpio_type].gpio_port, gpio_drv_cfg[gpio_type].gpio_pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(gpio_drv_cfg[gpio_type].gpio_port, gpio_drv_cfg[gpio_type].gpio_pin, GPIO_PIN_RESET);
	}
}

/*****************************************************************************
 函 数 名  : gpio_drv_read
 功能描述  : 获取GPIO值
 输入参数  : gpio_type_num gpio_type  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月24日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
U8 gpio_drv_read(gpio_type_num gpio_type)
{
	if(gpio_type >= GPIO_NUM)
	{
		ERR_PRINT(("GPIO参数非法!!!!\r\n"));
		return 0;
	}

	return HAL_GPIO_ReadPin(gpio_drv_cfg[gpio_type].gpio_port, gpio_drv_cfg[gpio_type].gpio_pin);
}

/*****************************************************************************
 函 数 名  : gpio_drv_init
 功能描述  : GPIO初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月22日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
void gpio_drv_init(void)
{
	U8 i = 0;
	GPIO_InitTypeDef gpio_initure;

	gpio_drv_set(GPIO_ISM330_SS, LEVEL_LOW);
	gpio_drv_set(GPIO_UBX_RST, LEVEL_HIGH);
	gpio_drv_set(GPIO_PANT, LEVEL_HIGH);
	gpio_drv_set(GPIO_FLASH_SS, LEVEL_LOW);
	//gpio_drv_set(GPIO_SIM_RST_PWR, LEVEL_HIGH);

	for(i = 0; i < GPIO_NUM; i++)
	{
		GLOBAL_MEMSET(&gpio_initure, 0x0, sizeof(gpio_initure));
		if(gpio_drv_cfg[i].gpio_mode == GPIO_MODE_INPUT)
		{
			gpio_initure.Mode = gpio_drv_cfg[i].gpio_mode;
			gpio_initure.Pin = gpio_drv_cfg[i].gpio_pin;
			gpio_initure.Pull = GPIO_NOPULL;
			gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
			HAL_GPIO_Init(gpio_drv_cfg[i].gpio_port, &gpio_initure);
		}
		else
		{
			gpio_initure.Mode = gpio_drv_cfg[i].gpio_mode;
			gpio_initure.Pin = gpio_drv_cfg[i].gpio_pin;
			gpio_initure.Pull = GPIO_PULLUP;//GPIO_PULLUP
			gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
			HAL_GPIO_Init(gpio_drv_cfg[i].gpio_port, &gpio_initure);
		}

		if(gpio_drv_cfg[i].irq_enable == TRUE)
		{
			gpio_initure.Mode = gpio_drv_cfg[i].gpio_mode;
			gpio_initure.Pin = gpio_drv_cfg[i].gpio_pin;
			gpio_initure.Pull = GPIO_NOPULL;
			gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
			HAL_GPIO_Init(gpio_drv_cfg[i].gpio_port, &gpio_initure);

			HAL_NVIC_SetPriority(gpio_drv_cfg[i].irqn, gpio_drv_cfg[i].prio, gpio_drv_cfg[i].sub_prio);       
			HAL_NVIC_DisableIRQ(gpio_drv_cfg[i].irqn);             //禁止中断
		}
	}
	//gpio_drv_set(GPIO_SZ157_S2, LEVEL_HIGH);
	//gpio_drv_set(GPIO_SZ157_S1, LEVEL_LOW);
	//gpio_drv_set(GPIO_AMP_CTRL, LEVEL_LOW);
	gpio_drv_set(GPIO_PHY_RESET, LEVEL_LOW);
	delay_ms(200);
	gpio_drv_set(GPIO_PHY_RESET, LEVEL_HIGH);
	delay_ms(200);
	gpio_drv_set(GPIO_13, LEVEL_LOW);//LED1 OPEN
	delay_ms(5);
	gpio_drv_set(GPIO_14, LEVEL_LOW);//LED10 OPEN
	delay_ms(5);
	gpio_drv_set(GPIO_15, LEVEL_LOW);//LED12 OPEN
	delay_ms(5);
	gpio_drv_set(GPIO_16, LEVEL_LOW);//LED14 OPEN
	delay_ms(5);
	gpio_drv_set(GPIO_17, LEVEL_LOW);//LED15 OPEN
	delay_ms(5);
	gpio_drv_set(GPIO_18, LEVEL_LOW);//LED15 OPEN
	delay_ms(5);
	//gpio_drv_set(GPIO_ISM330_INT1, LEVEL_LOW);
	NOTE_PRINT(("GPIO初始化完成!!!!!!\r\n"));
}

/*****************************************************************************
 函 数 名  : EXTI9_5_IRQHandler
 功能描述  : GPIO5-GPIO9的中断服务函数
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月14日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void EXTI9_5_IRQHandler(void)
{
  	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);  //调用中断处理公用函数
  	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);  //调用中断处理公用函数
}

/*****************************************************************************
 函 数 名  : EXTI15_10_IRQHandler
 功能描述  : GPIO10-GPIO15的中断服务函数
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月14日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void EXTI15_10_IRQHandler(void)
{
    //HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10); //调用中断处理公用函数
    if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_12))
	{
    	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12); //INT2:GY
    }
    if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_13))
    {
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13); //INT1:XL
    }
	if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_14))
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14); //INT/DRDY:MAG
	}
}
/*****************************************************************************
 函 数 名  : EXTI0_IRQHandler
 功能描述  : GPIO5-GPIO9的中断服务函数
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月14日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void EXTI0_IRQHandler(void)
{
	//暂用M8N代替
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);  //调用中断处理公用函数
}


//中断服务程序中需要做的事情
//在HAL库中所有的外部中断服务函数都会调用此函数
//GPIO_Pin:中断引脚号
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
    switch(GPIO_Pin)
    {
    	case GPIO_PIN_0:
			 //UBX_1PPS_time = osKernelGetTickCount();
			 break;
		case GPIO_PIN_5:
			 osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
             break;
    	case GPIO_PIN_6:
			 osEventFlagsSet(os_event_id_g, OS_EVENT_PPS);
			 break;
		case GPIO_PIN_10:
			 //threadSignalPost(THREAD_SIGNAL_BCODE_INT);
			 //time_update_flag = TRUE;
			 osEventFlagsSet(os_event_id_g, OS_EVENT_DOPLER_UPDATE);
			 break;
		case GPIO_PIN_12:
			 Imu_Time_Get(GPIO_PIN_12);
			 break;
		case GPIO_PIN_13:
			 Imu_Time_Get(GPIO_PIN_13);
			 break;
		case GPIO_PIN_14:
			 Imu_Time_Get(GPIO_PIN_14);
			 break;
		default:
			 break;
    }
}

/*****************************************************************************
 函 数 名  : gpio_enable_irq
 功能描述  : 使能中断
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月10日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gpio_enable_irq(void)
{
	U8 i = 0;
	
	for(i = 0; i < GPIO_NUM; i++)
	{
		if(gpio_drv_cfg[i].irq_enable == TRUE)
		{
			HAL_NVIC_EnableIRQ(gpio_drv_cfg[i].irqn);             
		}
	}
}


void gpio_turn_led(GPIO_TypeDef* port, U32 pin, uint8_t mode)
{

	HAL_GPIO_WritePin(port, pin, mode);
}


/*eof*/

