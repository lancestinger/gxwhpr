/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : main.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年5月29日
  最近修改   :
  功能描述   : 主程序文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

/*------------------------------头文件------------------------------*/
#include "drv/drv_init.h"
#include "fml/fml_init.h"
#include "apl/apl_init.h"
#include "gpio/gpio_drv.h"
//#include "EventRecorder.h"
/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/

/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
static osThreadId_t thread_init_id = 0;
static U64 thread_init_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_init_attr = {
  .stack_mem  = &thread_init_stk[0],
  .stack_size = sizeof(thread_init_stk),
  .priority = osPriorityNormal,
};
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/
static void _system_clock_config(void);
static void _mpu_config(void);
static void _cache_enable(void);
static void _thread_init(void* arg);
//static void _cpy_nvic_vector(void);
/*------------------------------函数声明------------------------------*/

/*****************************************************************************
 函 数 名  : main
 功能描述  : 主程序
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
int main(void)
 {
 	//_cpy_nvic_vector();
	/*MPU及CACHE配置*/
	_mpu_config();
	_cache_enable();
	
	/*HAL初始化*/
	HAL_Init();

	/*系统时钟配置*/
	_system_clock_config();
	 
	/*操作系统初始化*/
	osKernelInitialize ();

	thread_init_id = osThreadNew(_thread_init, NULL, &thread_init_attr);
	
	/*启动调度*/
	osKernelStart();
	
	while(1)
	{
		delay_ms(1000);
	}
}

/*****************************************************************************
 函 数 名  : _cpy_nvic_vector
 功能描述  : 拷贝中断向量表
 输入参数  : void  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月4日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
#if 0
static void _cpy_nvic_vector(void)
{
	U32 *SouceAddr = (uint32_t *)(FLASH_BANK1_BASE+VECT_TAB_OFFSET);
	U32 *DestAddr = (uint32_t *)D1_DTCMRAM_BASE;
	
	memcpy(DestAddr, SouceAddr, 0x400);
	/* 设置中断向量表到 ITCM 里面 */
	SCB->VTOR = D1_DTCMRAM_BASE;
}
#endif

/*****************************************************************************
 函 数 名  : _thread_init
 功能描述  : 主线程
 输入参数  : void* arg  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月31日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static void _thread_init(void* arg)
{
	/*驱动初始化*/
	drv_init();

	/*功能模块初始化*/
	fml_init();

	/*应用层初始化*/
	apl_init();

    GLOBAL_TRACE(("this is init thread!!!!\r\n"));
	
    GLOBAL_HEX(thread_init_id);
    GLOBAL_PRINT((ANSI_DARK_RED"\r\n\r\nsystem initialize is OK!!!!!\r\n\r\n\r\n"ANSI_NONE));
	osThreadTerminate(thread_init_id);

	while(1)
	{
		delay_ms(1000);
		// //gpio_enable_irq();
		// GLOBAL_TRACE(("this is init thread!!!!\r\n"));
		// GLOBAL_HEX(thread_init_id);
		// GLOBAL_PRINT((ANSI_DARK_RED"\r\n\r\nsystem initialize is OK!!!!!\r\n\r\n\r\n"ANSI_NONE));
		// osThreadTerminate(thread_init_id);  
	}
}

/*****************************************************************************
 函 数 名  : _cache_enable
 功能描述  : 使能CPU的L1-Cache
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
static void _cache_enable(void)
{
	SCB_EnableICache();//使能I-Cache
	SCB_EnableDCache();//使能D-Cache   
	SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题	
}

/*****************************************************************************
 函 数 名  : system_clock_config
 功能描述  : 系统时钟设置
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
static void _system_clock_config(void)
{

	RCC_OscInitTypeDef rcc_osc_initstruct;
	RCC_ClkInitTypeDef rcc_clk_initstruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	GLOBAL_MEMSET(&PeriphClkInitStruct, 0, sizeof(PeriphClkInitStruct));
	GLOBAL_MEMSET(&rcc_osc_initstruct, 0, sizeof(rcc_osc_initstruct));
	GLOBAL_MEMSET(&rcc_clk_initstruct, 0, sizeof(rcc_clk_initstruct));
	/**Supply configuration update enable 
	*/
	MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
	{

	}
	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	rcc_osc_initstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	rcc_osc_initstruct.HSEState = RCC_HSE_ON;
	rcc_osc_initstruct.PLL.PLLState = RCC_PLL_ON;
	rcc_osc_initstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	rcc_osc_initstruct.PLL.PLLM = 5;
	rcc_osc_initstruct.PLL.PLLN = 160;
	rcc_osc_initstruct.PLL.PLLP = 2;
	rcc_osc_initstruct.PLL.PLLQ = 5;
	rcc_osc_initstruct.PLL.PLLR = 2;
	rcc_osc_initstruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	rcc_osc_initstruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	rcc_osc_initstruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&rcc_osc_initstruct) != HAL_OK)
	{
	error_handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	rcc_clk_initstruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
	                          |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	rcc_clk_initstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	rcc_clk_initstruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	rcc_clk_initstruct.AHBCLKDivider = RCC_HCLK_DIV2;
	rcc_clk_initstruct.APB3CLKDivider = RCC_APB3_DIV2;
	rcc_clk_initstruct.APB1CLKDivider = RCC_APB1_DIV2;
	rcc_clk_initstruct.APB2CLKDivider = RCC_APB2_DIV2;
	rcc_clk_initstruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&rcc_clk_initstruct, FLASH_LATENCY_2) != HAL_OK)
	{
	error_handler(__FILE__, __LINE__);
	}

	//设置外设时钟
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMC | RCC_PERIPHCLK_SPI2;
	PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;	//选择AHB3时钟作为FMC时钟
	PeriphClkInitStruct.Spi123ClockSelection=RCC_SPI123CLKSOURCE_PLL;	//SPI123时钟源使用PLL1Q
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		ERR_PRINT(("外设时钟初始化失败!!!!\r\n"));
	}

	/* 使能GPIO的高速接口,需要使能IO补偿 */
	__HAL_RCC_CSI_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	HAL_EnableCompensationCell();
	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(SystemCoreClock/1000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/*****************************************************************************
 函 数 名  : _mpu_config
 功能描述  : MPU配置
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
static void _mpu_config(void)
{
  MPU_Region_InitTypeDef mpu_initstruct;

  /* Disables the MPU */
  HAL_MPU_Disable();
    /**Initializes and configures the Region and the memory to be protected 
    */
  mpu_initstruct.Enable = MPU_REGION_ENABLE;
  mpu_initstruct.Number = MPU_REGION_NUMBER0;
  mpu_initstruct.BaseAddress = 0x30040000;
  mpu_initstruct.Size = MPU_REGION_SIZE_256B;
  mpu_initstruct.SubRegionDisable = 0x0;
  mpu_initstruct.TypeExtField = MPU_TEX_LEVEL0;
  mpu_initstruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu_initstruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  mpu_initstruct.IsShareable = MPU_ACCESS_SHAREABLE;
  mpu_initstruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  mpu_initstruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&mpu_initstruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/*eof*/

