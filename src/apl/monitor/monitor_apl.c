/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : monitor_apl.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��5��30��
  ����޸�   :
  ��������   : ϵͳ��ش���
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��5��30��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


/*------------------------------ͷ�ļ�------------------------------*/
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

/*------------------------------ͷ�ļ�------------------------------*/

/*------------------------------�ļ���------------------------------*/
#define IWDG_ENABLE
#define SYS_UID_BASE			0x1FF1E800
/*------------------------------�ļ���------------------------------*/


/*------------------------------�ļ�����------------------------------*/
static U64 thread_monitor_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_monitor_attr = {
  .stack_mem  = &thread_monitor_stk[0],
  .stack_size = sizeof(thread_monitor_stk),
  .priority = osPriorityBelowNormal6,
};

static monitor_status_t monitor_handle;
static U32 const stat_max_idle_count = 99775300;	// 1���������м���
static volatile U32 stat_cur_idle_count = 0;	// 1���ڵ�ǰ���м���ֵ
static U8 stat_clear_flag = FALSE;
F32 cpu_usage_g = 0;		//CPUʹ����
U32 sysup_seconds_g = 0;	//ϵͳ�ϵ�ʱ��
F32 cpu_temper_g = 0;		//оƬ�¶�
static U32 err_code = 0;

u16 TAG_ID_SUM = 0;

extern U8 g_auto_time_flag;
/*------------------------------�ļ�����------------------------------*/

U16 TIMER_SHIFT = 0;
/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/
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
 �� �� ��  : osRtxIdleThread
 ��������  : �ض���ϵͳ���к���
 �������  : void *arg  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��31��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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

//�ض���rtxϵͳ������
U32 osRtxErrorNotify (U32 code, void *object_id) 
{
  (void)object_id;

  err_code = code;
  printf_type = 0;
  GLOBAL_PRINT(("������ = %u, ID = 0x%08X\r\n", code, object_id));
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
static IWDG_HandleTypeDef iwdg_handle; //�������Ź����
//��ʼ���������Ź�
//prer:��Ƶ��:IWDG_PRESCALER_4~IWDG_PRESCALER_256
//rlr:�Զ���װ��ֵ,0~0XFFF.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/32 (ms).


/*****************************************************************************
 �� �� ��  : monitor_thread
 ��������  : ����߳�
 �������  : void* arg  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��30��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
		HAL_IWDG_Refresh(&iwdg_handle); 	//ι��
		#endif
	}
}


/*****************************************************************************
 �� �� ��  : _iwdg_init
 ��������  : ������Ź���ʼ������
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��10��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
static void _iwdg_init(U8 prer, U16 rlr)
{
    iwdg_handle.Instance=IWDG1;
    iwdg_handle.Init.Prescaler=prer;	//����IWDG��Ƶϵ��
    iwdg_handle.Init.Reload=rlr;		//��װ��ֵ
    iwdg_handle.Init.Window=IWDG_WINDOW_DISABLE;//�رմ��ڹ���
    HAL_IWDG_Init(&iwdg_handle);		//��ʼ��IWDG  
	GLOBAL_TRACE(("���Ź���ʼ�����!!!!!\r\n"));
    //HAL_IWDG_Start(&iwdg_handle);		//�����������Ź�
}
#endif

/*****************************************************************************
 �� �� ��  : _monitor_timer
 ��������  : ��ʱ����߳�
 �������  : void* arg  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��31��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
	//GLOBAL_PRINT(("CPUʹ����:%.3f\r\n", cpu_usage_g));
}

/*****************************************************************************
 �� �� ��  : _generate_dev_sn
 ��������  : �����豸���кź�MAC��ַ����
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��10��
    ��    ��   : sunj
    �޸�����   : ����

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
 �� �� ��  : monitor_apl_init
 ��������  : ����̳߳�ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��30��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
	_iwdg_init(IWDG_PRESCALER_256, 0xFFF);  	//��Ƶ��Ϊ256,����ֵΪ0xFFF,���ʱ��Ϊ32s	
	#endif
}

