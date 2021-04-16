/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : gnss_fml.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��6��3��
  ����޸�   :
  ��������   : GNSS���ջ�����ģ��
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��6��3��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


/*------------------------------ͷ�ļ�------------------------------*/
#include "gnss/gnss_fml.h"
#include "data_rcv/data_rcv.h"
#include "uart/uart_fml.h"
#include "nmea.h"
#include "ubx.h"
#include "project_def.h"
/*------------------------------ͷ�ļ�------------------------------*/


/*------------------------------�ļ���------------------------------*/
#define NMEA_HEAD						'$'
#define NMEA_TAIL1						'\r'
#define NMEA_TAIL2						'\n'

#define UBX_HEAD1						0xB5
#define UBX_HEAD2						0x62

#define RTCM_HEAD1                      0xd3
#define RTCM_HEAD2                      0X0


#define GNSS_DATA_BUF_MAX_SIZE_PER_BUF	SIZE_1K
#define GNSS_DATA_BUF_MAX_INDEX			20
#define MAX_TMP_STR_BUF_LEN	31
/*------------------------------�ļ���------------------------------*/


/*------------------------------�ļ�����------------------------------*/
static U64 thread_gnss_stk[SIZE_2K / 8];
static const osThreadAttr_t thread_gnss_attr = {
  .stack_mem  = &thread_gnss_stk[0],
  .stack_size = sizeof(thread_gnss_stk),
  .priority = osPriorityNormal,
};

static UTL_DATA_RCV_T gnss_rcv_drv;
static U8 (*gnss_data_buf)[GNSS_DATA_BUF_MAX_SIZE_PER_BUF] = NULL;		//��ά����ָ��
static DATA_BUF_INFO_T gnss_data_buf_info[GNSS_DATA_BUF_MAX_INDEX];
static gnss_data_t gnss_data;
U32 gnss_condition_cnt = 0;		//GNSS�����жϼ���ֵ
U8 gnss_rcv_buf[GNSS_DATA_BUF_MAX_SIZE_PER_BUF];
static u8 RTCM_Cache[GNSS_DATA_BUF_MAX_SIZE_PER_BUF];
/*------------------------------�ļ�����------------------------------*/


/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/


/*****************************************************************************
 �� �� ��  : uart_fml_send_gnss_cmd
 ��������  : ����ubxЭ��
 �������  : U8* pbuf  
             U32 len   
 �������  : ��
 �� �� ֵ  : __weak
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��5��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
__weak void uart_fml_send_gnss_cmd(U8* pbuf, U32 len)
{
	uart_send_data(GNSS_COM, pbuf, len);
}

//���buffer
static void _gnss_data_clear_bufinfo(IN U8 release_buf, IN U32 index)
{
	if (index < GNSS_DATA_BUF_MAX_INDEX)
	{
		if (release_buf)
		{
			gnss_data_buf_info[index].in_use = FALSE;
		}
		else
		{
			gnss_data_buf_info[index].in_use = TRUE;
		}
		
		gnss_data_buf_info[index].buf_rcved = FALSE;
		gnss_data_buf_info[index].current_len = 0;
	}
}

//׼����һ������
static U8 _gnss_data_prepare_nextbuf(void)
{
	U32 index;

	for (index = 0; index < GNSS_DATA_BUF_MAX_INDEX; index++)
	{
		if (gnss_data_buf_info[index].in_use != TRUE)
		{
			break;
		}
	}

	if (index >= GNSS_DATA_BUF_MAX_INDEX)
	{
		gnss_rcv_drv.current_buf_info_index = -1;

		return FALSE;
	}

	gnss_rcv_drv.current_buf_info_index = index;

	/* For new prepared buf, should not release buf */
	_gnss_data_clear_bufinfo(FALSE, index);

	gnss_rcv_drv.buf_info = &gnss_data_buf_info[index];
	if(gnss_rcv_drv.buf_info->data_buf_ptr > gnss_data_buf[GNSS_DATA_BUF_MAX_INDEX-1])
	{
		ERR_PRINT(("��ַ���, (0x%08X > 0x%08X)\r\n", gnss_rcv_drv.buf_info->data_buf_ptr, gnss_data_buf[GNSS_DATA_BUF_MAX_INDEX-1]));
	}
	return TRUE;
}

//������ɺ�Ĳ���
static U8 _gnss_data_rcv_finishhandle(IN U8 *data_ptr, IN U32 len)
{

	/* received a valid data, Set Next buffer index */
	_gnss_data_prepare_nextbuf();

	return TRUE;	 
}

//���պ���
static BOOL _gnss_data_rcv_cb(IN CHAR_T data)
{
	if (gnss_rcv_drv.current_buf_info_index == -1)
	{
		/* No buf index, Prepare for next buffer */
		_gnss_data_prepare_nextbuf();
	}
	if(gnss_data.ubx_enable)
	{
		return UTL_DATA_RCV_Data_For_2Head0Tail(&gnss_rcv_drv, data);	
	}
	else if(gnss_data.mxt_enable)
	{
		return RTCM_DATA_RCV_Data_For_2Head0Tail(&gnss_rcv_drv, data);
	}
	else
	{
		return UTL_DATA_RCV_Data_For_1Head2Tail(&gnss_rcv_drv, data);	 
	}
}

static int _gnss_data_flush_rcvdata(unsigned char* buf)
{
	int i, ret = 0;

	for (i = 0; i < GNSS_DATA_BUF_MAX_INDEX; i++)
	{
		if (gnss_data_buf_info[i].buf_rcved)
		{
			if(gnss_data_buf_info[i].current_len < GNSS_DATA_BUF_MAX_SIZE_PER_BUF)
			{
				GLOBAL_MEMCPY(buf, gnss_data_buf_info[i].data_buf_ptr, gnss_data_buf_info[i].current_len);	
			}
			else
			{
				ERR_PRINT(("len = %d\r\n", gnss_data_buf_info[i].current_len));
			}
			ret = gnss_data_buf_info[i].current_len;
			_gnss_data_clear_bufinfo(TRUE, i);
			break;
		}
	}
	return ret;
	
}


//����ṹ��ʼ��
static void _gnss_init_datarcv(void)
{
	int i;
	
	for (i = 0; i < GNSS_DATA_BUF_MAX_INDEX; i++)
	{
		gnss_data_buf_info[i].data_buf_ptr = gnss_data_buf[i];
		//GLOBAL_HEX(gnss_data_buf_info[i].data_buf_ptr);
	}

	if(gnss_data.ubx_enable)
	{
		gnss_rcv_drv.max_size = GNSS_DATA_BUF_MAX_SIZE_PER_BUF;
		gnss_rcv_drv.rcv_status = DATA_RCV_NORMAL;
		gnss_rcv_drv.handle = (DATA_RCV_HANDLE_CB)_gnss_data_rcv_finishhandle;
		gnss_rcv_drv.header1 = UBX_HEAD1;
		gnss_rcv_drv.header2 = UBX_HEAD2;
		gnss_rcv_drv.current_buf_info_index = 0;
	}
	else if(gnss_data.mxt_enable)
	{
		gnss_rcv_drv.max_size = GNSS_DATA_BUF_MAX_SIZE_PER_BUF;
		gnss_rcv_drv.rcv_status = DATA_RCV_NORMAL;
		gnss_rcv_drv.handle = (DATA_RCV_HANDLE_CB)_gnss_data_rcv_finishhandle;
		gnss_rcv_drv.header1 = RTCM_HEAD1;
		gnss_rcv_drv.header2 = RTCM_HEAD2;
		gnss_rcv_drv.current_buf_info_index = 0;
	}
	else
	{
		gnss_rcv_drv.max_size = GNSS_DATA_BUF_MAX_SIZE_PER_BUF;
		gnss_rcv_drv.rcv_status = DATA_RCV_NORMAL;
		gnss_rcv_drv.handle = (DATA_RCV_HANDLE_CB)_gnss_data_rcv_finishhandle;
		gnss_rcv_drv.header1 = NMEA_HEAD;
		//gnss_rcv_drv.header2 = GNSS_HEAD2;
		gnss_rcv_drv.tail1 = NMEA_TAIL1;
		gnss_rcv_drv.tail2 = NMEA_TAIL2;
		gnss_rcv_drv.current_buf_info_index = 0;
	}
	/* prepare first buf */
	_gnss_data_prepare_nextbuf();
	
	
} 


/*****************************************************************************
 �� �� ��  : gnss_data_process
 ��������  : GNSS���洦��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2018��9��17�� ����һ
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
static int _gnss_data_process(void)
{
	int ret = 0, len = 0;
	static U32 gnss_rcv_loast_cnt = 0, run_count = 0,rtcm_cnt = 0, total_len=0;
	U16 anten_status = 0;
		
	run_count++;
	if((len = _gnss_data_flush_rcvdata(gnss_rcv_buf)) > 0)
	{
		if(gnss_data.ubx_enable)
		{
			gnss_ublox_ubx_data_process(gnss_rcv_buf, len);
			if(sys_debug_get_type(SYS_DEBUG_GNSS))
			{
				GLOBAL_PRINT(("UBX RCV[%u]:", len));
				for(U32 i = 0; i < len; i++)
				{
					if(i % 32 == 0)
					{
						GLOBAL_PRINT(("\r\n"));
					}
					GLOBAL_PRINT((" %02X", gnss_rcv_buf[i]));
				}
				GLOBAL_PRINT(("\r\n"));
			}
		}
		else if(gnss_data.mxt_enable)
		{
			rtcm_cnt++;//MQTT���ն˶�gnssģ�鲻��RTCM���մ���
		}
		else
		{
			gnss_nmea_data_process(gnss_rcv_buf, len);
			//DBG_GNSS_Print("%s", gnss_rcv_buf);
		}
		memset(gnss_rcv_buf, 0x0, sizeof(gnss_rcv_buf));
		APP_SET_BIT(gnss_data.receiver_criterion, GNSS_REF_CRITERION_RCV_DATA);
		//anten_status = fmc_read_reg(FMC_BASE_GPIO, BASE_IO_GPS_ANT_CHECK);//!adc_get_gnss_antenna_status();
		if(!anten_status)
		{
			APP_SET_BIT(gnss_data.receiver_criterion, GNSS_REF_CRITERION_ANTEN_STATUS);
		}
		else
		{
			APP_CLEAR_BIT(gnss_data.receiver_criterion, GNSS_REF_CRITERION_ANTEN_STATUS);
		}
		gnss_rcv_loast_cnt = 0;
		ret = 1;
	}
	else
	{
		gnss_rcv_loast_cnt++;
		if(gnss_rcv_loast_cnt > 600)
		{
			APP_CLEAR_BIT(gnss_data.receiver_criterion, GNSS_REF_CRITERION_RCV_DATA);
		}
	}
	return ret;
}

static void _gnss_thread(void* arg)
{
	CacheBuff.RX_flag = FALSE;
	while(1)
	{
		if(_gnss_data_process())
		{
			continue;
		}
		delay_ms(5);
	}
}

/*****************************************************************************
 �� �� ��  : gnss_fml_init
 ��������  : GNSSģ���ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��3��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void gnss_fml_init(INOUT gnss_data_t** p_handle)
{
	osThreadId_t thread_gnss_id = 0;
	
	GLOBAL_MEMSET(&gnss_data, 0x0, sizeof(gnss_data));
	GLOBAL_MEMSET(&gnss_rcv_drv, 0x0, sizeof(gnss_rcv_drv));
	GLOBAL_MEMSET(&gnss_data_buf_info, 0x0, sizeof(gnss_data_buf_info));
	memset(RTCM_Cache, 0x0, sizeof(RTCM_Cache));
	gnss_data.lat_ind = 'N';
	gnss_data.lon_ind = 'E';
	gnss_data_buf = (U8(*)[GNSS_DATA_BUF_MAX_SIZE_PER_BUF])GLOBAL_MALLOC(GNSS_DATA_BUF_MAX_INDEX*GNSS_DATA_BUF_MAX_SIZE_PER_BUF);
	if(!gnss_data_buf)
	{
		ERR_PRINT(("gnss_data_buf����ʧ��!!!!\r\n"));
		return;
	}
	gnss_data.ubx_enable = main_handle_g.cfg.ubx_enable;
	*p_handle = &gnss_data;

//-----------CLOSE UBX RECV----------//
	gnss_data.ubx_enable = 0;
//-----------OPEN RTCM RECV----------//
	gnss_data.mxt_enable = 0;
//-----------------------------------//
	GLOBAL_INTVAL(gnss_data.ubx_enable);
	
	//gnss_ubx_init(&gnss_data);
	gnss_mxt_init();
		//Origin_flag = 0;      

	gnss_nmea_init(&gnss_data);
	
	_gnss_init_datarcv();
	uart_register_rx_callbak(UART_COM4, _gnss_data_rcv_cb);	//ע����Դ���
	thread_gnss_id = osThreadNew(_gnss_thread, NULL, &thread_gnss_attr);
	GLOBAL_HEX(thread_gnss_id);
}

/*eof*/
