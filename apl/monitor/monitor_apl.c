/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : monitor_apl.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年5月30日
  最近修改   :
  功能描述   : 系统监控处理
  函数列表   :
  修改历史   :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


/*------------------------------头文件------------------------------*/
#include "monitor/monitor_apl.h"
#include "uart/uart_fml.h"
#include "project_def.h"
#include "adc/adc_drv.h"
#include "socket/socket_drv.h"
#include "gnss/gnss_fml.h"
#include "gnss/ubx.h"
#include "crc/crc.h"
#include "gpio/gpio_drv.h"
//#include "protocol/protocol_apl.h"

/*------------------------------头文件------------------------------*/

/*------------------------------文件宏------------------------------*/
#define IWDG_ENABLE
#define SYS_UID_BASE			0x1FF1E800
/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
static U64 thread_monitor_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_monitor_attr = {
  .stack_mem  = &thread_monitor_stk[0],
  .stack_size = sizeof(thread_monitor_stk),
  .priority = osPriorityBelowNormal6,
};

static monitor_status_t monitor_handle;
static U32 const stat_max_idle_count = 99775300;	// 1秒内最大空闲计数
static volatile U32 stat_cur_idle_count = 0;	// 1秒内当前空闲计数值
static U8 stat_clear_flag = FALSE;
F32 cpu_usage_g = 0;		//CPU使用率
U32 sysup_seconds_g = 0;	//系统上电时间
F32 cpu_temper_g = 0;		//芯片温度
static U32 err_code = 0;

u16 TAG_ID_SUM = 0;

extern U8 g_auto_time_flag;
/*------------------------------文件变量------------------------------*/

U16 TIMER_SHIFT = 0;
/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/
#if 0
    fsStatus fs_get_time (fsTime *time) 
    {
        //..
        // Real time should be read from the RTC
        //..
        // Fill the FS_TIME structure with the time information
        time->hr   = 12;    // Hours:   0 - 23
        time->min  = 0;     // Minutes: 0 - 59
        time->sec  = 0;     // Seconds: 0 - 59

        time->day  = 1;     // Day:     1 - 31
        time->mon  = 1;     // Month:   1 - 12
        time->year = 2019;  // Year:    1980 - 2107
        
        GLOBAL_PRINT(("################################################fs_get_time\r\n"));
        return (fsOK);
    }
#endif
/*****************************************************************************
 函 数 名  : osRtxIdleThread
 功能描述  : 重定义系统空闲函数
 输入参数  : void *arg  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月31日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void osRtxIdleThread (void *arg) {
  (void)arg;

  for (;;) 
  {
  	if(stat_clear_flag)
  	{
  		stat_clear_flag = FALSE;
		stat_cur_idle_count = 0;
  	}
  	stat_cur_idle_count++;
  }
}

//重定义rtx系统错误函数
U32 osRtxErrorNotify (U32 code, void *object_id) 
{
  (void)object_id;

  err_code = code;
  printf_type = 0;
  GLOBAL_PRINT(("错误码 = %u, ID = 0x%08X\r\n", code, object_id));
  switch (err_code) {
    case osRtxErrorStackUnderflow:  //1U
      // Stack overflow detected for thread (thread_id=object_id)
      break;
    case osRtxErrorISRQueueOverflow:    //2U    
      // ISR Queue overflow detected when inserting object (object_id)
      break;
    case osRtxErrorTimerQueueOverflow:  //3U
      // User Timer Callback Queue overflow detected for timer (timer_id=object_id)
      break;
    case osRtxErrorClibSpace:   //4U
      // Standard C/C++ library libspace not available: increase OS_THREAD_LIBSPACE_NUM
      break;
    case osRtxErrorClibMutex:   //5U
      // Standard C/C++ library mutex initialization failed
      break;
    default:
      // Reserved
      break;
  }
  for (;;) {}
//return 0U;
}


#ifdef IWDG_ENABLE
static IWDG_HandleTypeDef iwdg_handle; //独立看门狗句柄
//初始化独立看门狗
//prer:分频数:IWDG_PRESCALER_4~IWDG_PRESCALER_256
//rlr:自动重装载值,0~0XFFF.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).


/*****************************************************************************
 函 数 名  : monitor_thread
 功能描述  : 监控线程
 输入参数  : void* arg  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static void _monitor_thread(void* arg)
{
	U32 count = 0;
    U32 tickCnt = 0;

	
	while(1)
	{
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
		delay_ms(500);
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
        delay_ms(500);

		#ifdef IWDG_ENABLE
		HAL_IWDG_Refresh(&iwdg_handle); 	//喂狗
		#endif
	}
}


/*****************************************************************************
 函 数 名  : _iwdg_init
 功能描述  : 软件看门狗初始化函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年10月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
static void _iwdg_init(U8 prer, U16 rlr)
{
    iwdg_handle.Instance=IWDG1;
    iwdg_handle.Init.Prescaler=prer;	//设置IWDG分频系数
    iwdg_handle.Init.Reload=rlr;		//重装载值
    iwdg_handle.Init.Window=IWDG_WINDOW_DISABLE;//关闭窗口功能
    HAL_IWDG_Init(&iwdg_handle);		//初始化IWDG  
	GLOBAL_TRACE(("看门狗初始化完成!!!!!\r\n"));
    //HAL_IWDG_Start(&iwdg_handle);		//开启独立看门狗
}
#endif

/*****************************************************************************
 函 数 名  : _monitor_timer
 功能描述  : 定时监控线程
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
static void _monitor_timer(void* arg)
{
	static U32 count = 0;

	count++;
	
	
	if(count % 2 == 0)
	{
		if(stat_cur_idle_count <= stat_max_idle_count)
		{
			cpu_usage_g = (stat_max_idle_count - stat_cur_idle_count)*100.0/stat_max_idle_count;
		}
		stat_clear_flag = TRUE;
		sysup_seconds_g++;
	}
	//GLOBAL_PRINT(("CPU使用率:%.3f\r\n", cpu_usage_g));
}

/*****************************************************************************
 函 数 名  : _generate_dev_sn
 功能描述  : 产生设备序列号和MAC地址函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年10月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
static void _generate_dev_sn(void)
{
	U8 i = 0;
	U8 u_id[12];
	U32 crc_32 = 0;
	U16 crc_16 = 0;
	U8* p_sn = monitor_handle.dev_sn;
	U8* p_mac = monitor_handle.dev_mac;
	
	for(i = 0;i < 12;i++)
	{
		u_id[i] = *(U8 *)(SYS_UID_BASE + i); 
		GLOBAL_PRINT(("%02X ", u_id[i]));
		if(i == 11)
		{
			GLOBAL_PRINT(("\r\n"));
		}
	}
	crc_32 = crc32(u_id, 12);
	crc_16 = crc16(u_id, 12);
	GLOBAL_HEX(crc_32);
	GLOBAL_HEX(crc_16);
	i = 0;
	p_sn[i++] = 'H';
	p_sn[i++] = 'P';
	p_sn[i++] = 'R';
	p_sn[i++] = 0x01;
	p_sn[i++] = (crc_32 >> 24)&0xFF;
	p_sn[i++] = (crc_32 >> 16)&0xFF;
	p_sn[i++] = (crc_32 >> 8)&0xFF;
	p_sn[i++] = (crc_32 >> 0)&0xFF;

	i = 0;
	p_mac[i++] = 0x00;
	p_mac[i++] = 0x5E;
	p_mac[i++] = 0x33;
	p_mac[i++] = 0x01;
	p_mac[i++] = (crc_16 >> 8)&0xFF;
	p_mac[i++] = (crc_16 >> 0)&0xFF;
}


static void Tag_id_combine(void)
{
	u16 num_1 = 0, num_2 = 0;
	
	num_1 = (monitor_handle.dev_sn[4]<<8)|(monitor_handle.dev_sn[5]);
	num_2 = (monitor_handle.dev_sn[6]<<8)|(monitor_handle.dev_sn[7]);
	TAG_ID_SUM = num_1 + num_2;
	GLOBAL_PRINT(("TAG_ID_SUM = %d\r\n",TAG_ID_SUM));
}


/*****************************************************************************
 函 数 名  : monitor_apl_init
 功能描述  : 监控线程初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void monitor_apl_init(void)
{
	osThreadId_t thread_monitor_id = 0;
	osTimerId_t timer_monitor_id = 0;

	GLOBAL_MEMSET(&monitor_handle, 0x0, sizeof(monitor_handle));

	main_handle_g.p_monitor = &monitor_handle;
	_generate_dev_sn();
	Tag_id_combine();
	thread_monitor_id = osThreadNew(_monitor_thread, NULL, &thread_monitor_attr);
	GLOBAL_HEX(thread_monitor_id);

	timer_monitor_id = osTimerNew(_monitor_timer, osTimerPeriodic, NULL, NULL);
	GLOBAL_HEX(timer_monitor_id);
	if(timer_monitor_id)
	{
		osStatus_t res = osTimerStart(timer_monitor_id, 5000);
		GLOBAL_INTVAL(res);
	}
	sys_net_cfg_start();
	#ifdef IWDG_ENABLE
	_iwdg_init(IWDG_PRESCALER_256, 0xFFF);  	//分频数为256,重载值为0xFFF,溢出时间为32s	
	#endif
}

