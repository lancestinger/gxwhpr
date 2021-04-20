/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : nmea.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��6��3��
  ����޸�   :
  ��������   : NMEA��������
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��6��3��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


/*------------------------------ͷ�ļ�------------------------------*/
#include "gnss/nmea.h"
#include "comm/sys_debug.h"
#include "apl/imu/Imu.h"
#include "project_def.h"
#include "drv/uart/uart_drv.h"
#include "json/Cjson_run.h"
#include "apl/imu/ins_baseparams.h"
#include "apl/imu/Coordi_transfer.h"
#include "apl/server/server_apl.h"
#include "drv/meas/meas_distance.h"

/*------------------------------ͷ�ļ�------------------------------*/


/*------------------------------�ļ���------------------------------*/

#define MAX_TMP_STR_BUF_LEN	31
typedef enum
{
	GNSS_NMEA_MSG_GGA,
	GNSS_NMEA_MSG_RMC,
	GNSS_NMEA_MSG_ZDA,
	GNSS_NMEA_MSG_GSV,
	GNSS_NMEA_MSG_GSA,
	GNSS_NMEA_MSG_PUBX,

	GNSS_NMEA_MSG_END,
}GNSS_NMEA_MSG_ENUM;

typedef struct
{
	GNSS_NMEA_MSG_ENUM msg_type;
	char *msg_header;
}GNSS_NMEA_MSG_MAP_T;

 /*���ջ��ж�����*/
/*#define GNSS_CRITERION_VALID     ( (1<< GNSS_REF_CRITERION_POSITION_FIX) |\
										 (1<< GNSS_REF_CRITERION_LEAP_FLAG) |\
										 (1<< GNSS_REF_CRITERION_SEC_CONTINUES) |\
										 (1 << GNSS_REF_CRITERION_ANTEN_STATUS) |\
										 (1<< GNSS_REF_CRITERION_RCV_DATA))		
*/


#define JUDGE_SIGNAL_INTENSITY_MIN                              29		/*��С�����*/										 
/*------------------------------�ļ���------------------------------*/


/*------------------------------�ļ�����------------------------------*/
static gnss_data_t* p_gnss_handle = NULL;
U8 GGA_OLD_FLAG = FALSE;
static const GNSS_NMEA_MSG_MAP_T gnss_nmea_msg_map[GNSS_NMEA_MSG_END] =
{
	/* msg type,            	msg_header */
	{GNSS_NMEA_MSG_GGA, 	"GGA,"},
    {GNSS_NMEA_MSG_RMC, 	"RMC,"},
    {GNSS_NMEA_MSG_ZDA, 	"ZDA,"},
	{GNSS_NMEA_MSG_GSV, 	"GSV,"},
	{GNSS_NMEA_MSG_GSA, 	"GSA,"},
	{GNSS_NMEA_MSG_PUBX, 	"UBX,"},
};

//static U8 pubx_recv_flag = FALSE;
/*------------------------------�ļ�����------------------------------*/

Parse_data NMEA_Data_ptr;//
char Sys_Date[20]={0};
char Sys_UTC[20]={0};

U32 UBX_1PPS_time = 0;//GPS���ջ�1PPSʱ�����
U8 Origin_flag = 0;//����ת��ԭ���־
U8 GGA_DATA_READY = 0;//GGA���ݲɼ���ϱ�־
U8 RMC_DATA_READY = 0;//RMC���ݲɼ���ϱ�־
U8 NMEA_OPEN_SWITCH = 0;


/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/

/*****************************************************************************
 �� �� ��  : _utl_hex2i
 ��������  : hexת10������
 �������  : U8 str  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��3��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
static U32 _utl_hex2i(U8 str)
{
	U8			ch;
	U32 	val = 0;

	ch = str;
	if ((ch >='0') && (ch <='9'))
		val |=  (ch-'0');
	else
	{
		if ((ch >='a') && (ch <='f'))
			val |=  (ch -'a'+10);
		else
		{
			if ((ch >='A') && (ch <='F'))
				val |=  (ch -'A'+10);
		}
	}

	return val;
}

/*****************************************************************************
 �� �� ��  : _utl_xor
 ��������  : ����ۼ�
 �������  : U8 *data  
             U32 len   
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��3��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
static U8 _utl_xor(U8 *data, U32 len)
{
    U8 xor_value = 0;

    if(len < 2)
    {
        return 0;
    }

    while(len--)
    {
        xor_value  ^= *data++;
    }

	return xor_value;
}


/*****************************************************************************
 �� �� ��  : _nmea_data_checksum
 ��������  : NMEA����У���
 �������  : IN U8 *data_ptr  
             IN U32 len       
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��3��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
BOOL _nmea_data_checksum(IN U8 *data_ptr, IN U32 len)
{
	U8 xor_checksum = 0;
	U8 xor_checksum_inmsg = 0;

	/**************************************************************************
	*
	*	Data Format: $EXTI,<1>,<2>,<3>,<3>,<4>,<6>*hh<CR><LF>
	*					$AACCC��c��c*hh<CR><LF>
	*		hh: XOR value between $ and *, except $ and *
	*
	***************************************************************************/
	if((len<7)||(*(data_ptr + len - 5) != '*'))
    {
        return FALSE;
    }

	xor_checksum_inmsg = _utl_hex2i(*(data_ptr + len - 4));
	xor_checksum_inmsg <<= 4;
	xor_checksum_inmsg += _utl_hex2i(*(data_ptr + len - 3));

	/* Calc Data check sum */
	xor_checksum = _utl_xor(data_ptr + 1, len - 6);

	if (xor_checksum != xor_checksum_inmsg)
	{
		ERR_PRINT(("\r\n CKS: %02x, %02x", xor_checksum_inmsg, xor_checksum));

		return FALSE;
	}

	return TRUE;
}

/*****************************************************************************
 �� �� ��  : _gnss_nmea_get_data_before_comma
 ��������  : ��ȡnmea����ǰ������
 �������  : IN U8 *str                         
             OUT U8 *folling_str_between_comma  
             IN U32 max_cnt                     
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2018��9��18�� ���ڶ�
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
static U8 _gnss_nmea_get_data_before_comma(IN U8 *str, OUT U8 *folling_str_between_comma, IN U32 max_cnt)
{
    U8 cnt = 0;

    while(*str != '\0')
    {
        if (*str != ',')
        {
        	if (cnt >= max_cnt)
        	{
   				return cnt;
        	}
            *folling_str_between_comma++ = *str;
            cnt ++;
        }
        else
        {
            break;
        }
        str++;
    }

    return cnt;
}

/*****************************************************************************
 �� �� ��  : _gnss_nmea_get_data_after_comma
 ��������  : ��ȡ���ź������
 �������  : IN U8 *msg       
             IN U8 comma_num  
 �������  : ��
 �� �� ֵ  : U8
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2018��9��18�� ���ڶ�
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
static U8* _gnss_nmea_get_data_after_comma(IN U8 *msg, IN U8 comma_num)
{
    U8 comma_count = 0;

    if(comma_num == 0)
    {
        return msg;
    }

    while(*msg != '*')
    {
        if(*msg++ == ',')
        {
            comma_count++;
        }

        if(comma_count == comma_num)
        {
            return msg;
        }
    }
	return NULL;
}

//����GGA���
static void _gnss_parse_gga(IN U8 *data_ptr,IN U32 len)
{
	U32 tmp_int;
    F32 tmp_float;
    U8 str_buf[MAX_TMP_STR_BUF_LEN + 1];
    U8 *str_buf_ptr;
    U8 str_cnt = 0;
	U8 ref_valid_flag = FALSE;
    //U16 anten_status = 0;
#if 1
    /* ȡ��λ��Ϣ */
	DBG_SERVER_Print("%s\r\n",data_ptr);
	//GLOBAL_PRINT(("GGA_RX_flag = %d\r\n",CacheBuff.RX_flag));

    str_buf_ptr = _gnss_nmea_get_data_after_comma(data_ptr,6);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_buf[0] == '0')
    {
    	GGA_OLD_FLAG = TRUE;
		SERVER_RTK = POS_INVALID;//�ر�RTK���ݱ�־
		
		NMEA_Data_ptr.RTK_mode = 0;
        APP_CLEAR_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_POSITION_FIX);
    }
    else
    {	
    	SERVER_RTK = POS_VALID;
    	GGA_OLD_FLAG = FALSE;
   		if(CacheBuff.RX_flag == FALSE)
		{
			//GLOBAL_MEMSET(CacheBuff.RX_pData,0x0,RX_LEN);
			GLOBAL_MEMCPY(CacheBuff.RX_pData,data_ptr,len);
			CacheBuff.RX_Size = len;
			CacheBuff.RX_flag = TRUE;		
		}
        APP_SET_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_POSITION_FIX);
		switch (str_buf[0])
		{
			case '1':
				NMEA_Data_ptr.RTK_mode = 1;
				break;
			case '2':
				NMEA_Data_ptr.RTK_mode = 2;
				break;
			case '4':
				NMEA_Data_ptr.RTK_mode = 4;
				break;
			case '5':
				NMEA_Data_ptr.RTK_mode = 5;
				break;
			default:
				break;
		}
    }

    /* ȡ���������� */
    str_buf_ptr=_gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
        GLOBAL_SSCANF((char *)str_buf, "%d", &tmp_int);
        p_gnss_handle->use_sat_num = tmp_int;
    }
    else
    {
        p_gnss_handle->use_sat_num = 0;
    }
#if 0
    ublox_info.satellites = UTL_HexToI(str_buf[0]);
    ublox_info.satellites <<= 4;
    ublox_info.satellites = UTL_HexToI(str_buf[1]);
#endif
    /* ȡHDOP */
    str_buf_ptr=_gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
        //GLOBAL_SSCANF((char *)str_buf, "%f", &p_gnss_handle->hdop);
    }
    else
    {
        p_gnss_handle->pdop = 0;
    }

    /* ȡ�߶� */
    str_buf_ptr=_gnss_nmea_get_data_after_comma(str_buf_ptr, 1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
        GLOBAL_SSCANF((char *)str_buf, "%f", &p_gnss_handle->altitude);
		GLOBAL_SSCANF((char *)str_buf, "%f", &NMEA_Data_ptr.alt);
    }
   /* else
    {
    	p_gnss_handle->altitude = 0;
    }*/

    /* ȡγ�� */
    str_buf_ptr=_gnss_nmea_get_data_after_comma(data_ptr,2);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
        GLOBAL_SSCANF((char *)str_buf, "%02d%f", &tmp_int, &tmp_float);

		p_gnss_handle->lattitude = tmp_int+tmp_float/60;
		NMEA_Data_ptr.lat = tmp_int+tmp_float/60;
	}
    /*else
    {
		p_gnss_handle->lattitude = 0;
    }*/

    /* ȡ N/S N-��γ��S-��γ */
    str_buf_ptr=_gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
        p_gnss_handle->lat_ind = str_buf[0];
    }
	if(str_buf[0] == 'S'||str_buf[0] == 's')
	{
		NMEA_Data_ptr.lat = (NMEA_Data_ptr.lat)*(-1.0);
	}

    /* ȡ���� */
    str_buf_ptr=_gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
        GLOBAL_SSCANF((char *)str_buf, "%03d%f", &tmp_int, &tmp_float);

        p_gnss_handle->lontitude = tmp_int+tmp_float/60;
		NMEA_Data_ptr.lon = tmp_int+tmp_float/60;
    }
    /*else
    {
    	p_gnss_handle->lontitude = 0;
    }*/

    /* ȡ E-������W-���� */
    str_buf_ptr=_gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
        p_gnss_handle->lon_ind = str_buf[0];
    }
 	if(str_buf[0] == 'W'||str_buf[0] == 'w')
	{
		NMEA_Data_ptr.lon = (NMEA_Data_ptr.lon)*(-1.0);
	}

    if((p_gnss_handle->receiver_criterion & GNSS_CRITERION_VALID) == GNSS_CRITERION_VALID)
    {
    	ref_valid_flag = TRUE;
    }
    else
    {
    	ref_valid_flag = FALSE;
    }

	GGA_DATA_READY++;

	
	/* Check machine status */
	//main_handle_g.p_ref_handle->ref_status[REF_GNSS] = ref_check_valid(REF_GNSS, &gnss_condition_cnt, ref_valid_flag);
	/*mult_handle_g.ref_handle.ref_status[REF_GNSS] = sys_ref_check_valid(REF_GNSS,
									&ublox_condition_cnt, ref_valid_flag);
	*/
	//GNSS_REF_CORE_Updata_ExtRefInfo(GNSS_REF_TYPE_GPS, ublox_info.machine_status, &ublox_info, FALSE);
#endif

}

//RMC����
static void _gnss_parse_rmc(IN U8 *data_ptr)
{
    U8 str_buf[MAX_TMP_STR_BUF_LEN + 1];
	U32 Date_ptr=0;
	U32 UTC_ptr = 0;
	F32 Angle = 0;
	F32 Veloc = 0;
    U8 *str_buf_ptr;
    U8 str_cnt = 0;
	U32 tmp_int[10];

	DBG_SERVER_Print("%s\r\n",data_ptr);

	//ȡUTCʱ��//
	str_buf_ptr = _gnss_nmea_get_data_after_comma(data_ptr,1);
	str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
	str_buf[str_cnt] = '\0';
	if(str_cnt)
	{
		GLOBAL_SSCANF((char *)str_buf,"%06d",&UTC_ptr);
		sprintf(Sys_UTC,"%06d",UTC_ptr);
		//GLOBAL_PRINT(("UTC time = %s\r\n",Sys_UTC));
		UTC_ptr = 0;
	}


	//ȡ����//
	str_buf_ptr = _gnss_nmea_get_data_after_comma(data_ptr,7);
	str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
	str_buf[str_cnt] = '\0';
	if(str_cnt)
	{
		GLOBAL_SSCANF((char *)str_buf,"%f",&Veloc);
		NMEA_Data_ptr.veloc = (double)(Veloc*KNToMS);
		//GLOBAL_PRINT(("NMEA Veloc  = %f\r\n",Veloc));		
		//GLOBAL_PRINT(("NMEA Veloc Get = %f\r\n",NMEA_Data_ptr.veloc));
		VELOC_VALID = 1;
		Veloc=0;
	}
	else
	{
		VELOC_VALID = 0;
	}
	
	//ȡ�����//
	str_buf_ptr = _gnss_nmea_get_data_after_comma(str_buf_ptr,1);
	str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
	str_buf[str_cnt] = '\0';
	if(str_cnt)
	{
		FIRST_ANGLE_GET = TRUE;
		GLOBAL_SSCANF((char *)str_buf,"%f",&Angle);
		NMEA_Data_ptr.angle = Angle;
		//GLOBAL_PRINT(("NMEA Angle Get = %f\r\n",NMEA_Data_ptr.angle));
		Angle=0;
	}


	//ȡ����//
	str_buf_ptr = _gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
	if(str_cnt)
	{
		GLOBAL_SSCANF((char *)str_buf,"%02d%02d%02d",&tmp_int[0],&tmp_int[1],&tmp_int[2]);
		Date_ptr = tmp_int[2]*10000+tmp_int[1]*100+tmp_int[0];
		sprintf(Sys_Date,"%06d",Date_ptr);
		//GLOBAL_PRINT(("DATE = %s\r\n",Sys_Date));
		Date_ptr = 0;
	}

	RMC_DATA_READY++;

}


//GSA����
static void _gnss_parse_gsa(IN U8 *data_ptr)
{
    U8 str_buf[MAX_TMP_STR_BUF_LEN + 1];
    U8 *str_buf_ptr;
    U8 str_cnt = 0;
	//U32 tmp_int = 0;
	//float tmp_float;

    /* PDOP */
    str_buf_ptr = _gnss_nmea_get_data_after_comma(data_ptr,15);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
    	GLOBAL_SSCANF((char *)str_buf, "%f", &p_gnss_handle->pdop);
    }
    else
    {
		p_gnss_handle->pdop = 99.99;
    }

	/* HDOP */
	str_buf_ptr = _gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
    	GLOBAL_SSCANF((char *)str_buf, "%f", &p_gnss_handle->hdop);
    }
    else
    {
		p_gnss_handle->hdop = 99.99;
    }

	/* VDOP */
	str_buf_ptr = _gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
    	GLOBAL_SSCANF((char *)str_buf, "%f", &p_gnss_handle->vdop);
    }
    else
    {
		p_gnss_handle->vdop = 99.99;
    }
	

}

//ZDA����
static void _gnss_parse_zda(IN U8 *data_ptr)
{
	//U32 tmp_int[10];
	//int fetch_num = 0;
   // static U8 pre_seconds = 0;
#if 0
    /*���ջ���������*/
    TIME_API_DateTime_Add1S(&ublox_info.ref_time);

	/*
	*	$GPZDA,082710.00,16,09,2002,00,00*64
	*/
	fetch_num = GLOBAL_SSCANF((char *)(data_ptr + 7),
						"%02d%02d%02d.%*2u,"	/* <1>  UTC Time: UTCʱ�䣬ǰ��λ����ʾСʱ����������λ��ʾ���ӣ������ʾ�� */
						"%02d,"					/* <2>  Day: �գ�01��31��*/
						"%02d,"					/* <3>  Month: �£�01��12��*/
						"%04d,"					/* <4>  Year: ��2012  */
						"%d,"					/* <5>  Local zone hour  ����ʱ����Сʱ��-13��+13��*/
						"%d,",				/* <6>  Local zone minutes ����ʱ�������ӣ�00��59��*/
						&tmp_int[0],&tmp_int[1],&tmp_int[2],
						&tmp_int[3],
						&tmp_int[4],
						&tmp_int[5],
						&tmp_int[6],
						&tmp_int[7]);

	if (fetch_num != 8)
	{
		return;
	}

	if((pre_seconds+1)%60 == tmp_int[2])
	{
		APP_SET_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_SEC_CONTINUES);
	}
	else
	{
		APP_CLEAR_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_SEC_CONTINUES);
	}
	pre_seconds = tmp_int[2];
	p_gnss_handle->time.year = tmp_int[5];
	p_gnss_handle->time.month = tmp_int[4];
	p_gnss_handle->time.mday = tmp_int[3];

    p_gnss_handle->time.hour = tmp_int[0];
	p_gnss_handle->time.min = tmp_int[1];
	p_gnss_handle->time.sec = tmp_int[2];
	// ref_cpy_time(REF_GNSS, &(p_gnss_handle->time));
	//EXT_REF_CORE_Updata_ExtRefInfo(EXT_REF_TYPE_GPS, ublox_info.machine_status, &ublox_info, TRUE);
#endif
}

//GSV������
static void _gnss_parse_gsv(IN U8 *data_ptr)
{
    U8 str_buf[MAX_TMP_STR_BUF_LEN + 1];
    U8 *str_buf_ptr;
    U32 int_tmp= 0;
    U8 str_cnt = 0;
    U8 ret =0;
    U32 num_in_row = 0;

    static U32 valid_cno = 0;
	static U32 gsv_rows  = 0;    /*�������*/
    static U32 gsv_index = 0;    /*�ڼ������*/
    static U32 gsv_num = 0;      /*�ڼ������*/
    U32 sig_cno = 0, sig_prn = 0;		        /*�����*/
    U8 i;


    /* ȡ�ɼ������� */
    str_buf_ptr=_gnss_nmea_get_data_after_comma(data_ptr,3);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
        GLOBAL_SSCANF((char *)str_buf, "%d", &int_tmp);//ublox_info.satellites
        p_gnss_handle->view_sat_num = int_tmp;
    }
    else
    {
        int_tmp = 0;
		p_gnss_handle->view_sat_num = 0;
    }

    /*ȡ�����*/
	str_buf_ptr=_gnss_nmea_get_data_after_comma(data_ptr,1);
    ret= GLOBAL_SSCANF((char *)str_buf_ptr, "%u, %u, %u", &gsv_rows, &gsv_index, &gsv_num);

    if((ret != 3) || (0 == gsv_rows) || (0 == gsv_num))
    {
        return ;
    }

    if(1 == gsv_index)
    {
        valid_cno = 0;
    }

    if(gsv_index < gsv_rows)
    {
        num_in_row = 4;
    }
    else
    {
        num_in_row = gsv_num - 4*(gsv_rows - 1);
    }

    str_buf_ptr=_gnss_nmea_get_data_after_comma(data_ptr,4);
	for(i = 0; i < num_in_row; i++)
    {
    
        ret = GLOBAL_SSCANF((char *)str_buf_ptr, "%u,%*u,%*u,%u",&sig_prn, &sig_cno); /*�����*/

		//GLOBAL_INTVAL(sig_prn); GLOBAL_INTVAL(sig_cno);
        if(sig_cno > JUDGE_SIGNAL_INTENSITY_MIN)
        {
            ++valid_cno;
        }
		str_buf_ptr=_gnss_nmea_get_data_after_comma(str_buf_ptr,4);
    }

	if(gsv_index == gsv_rows)
	{
		//printf("valid_cno:%d",valid_cno);
		p_gnss_handle->db30_sat_num = valid_cno;
		if(valid_cno>3)
		{
            APP_SET_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_CNO_STATUS);
        }
        else
        {
            APP_CLEAR_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_CNO_STATUS);
        }
	}

	//EXT_REF_Updata_ExtRefInfo(type, ublox_info.machine_status, &ublox_info, FALSE);
}

//PUBX����
static void _gnss_ublox_parse_pubx(IN U8 *data_ptr)
{
    U8 str_buf[MAX_TMP_STR_BUF_LEN + 1];
    U8 *str_buf_ptr;
    U8 str_cnt = 0, tmp_char = 0;
	U32 tmp_int = 0;
	F32 tmp_f = 0;
	
	//float tmp_float;

    /* PDOP */
    str_buf_ptr = _gnss_nmea_get_data_after_comma(data_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
    	GLOBAL_SSCANF((char *)str_buf, "%u", &tmp_int);
		if(tmp_int != 0x04)		//�����������־���
		{
			return;
		}
    }
    else
    {
		return;
    }

	/* ȡ������ */
	str_buf_ptr = _gnss_nmea_get_data_after_comma(data_ptr,4);
	str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
	str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
		if(GLOBAL_SSCANF((char *)str_buf, "%f", &tmp_f) == 1)
		{
			p_gnss_handle->gps_tow = tmp_f;
		}
	}
	/* ȡ���� */
	str_buf_ptr = _gnss_nmea_get_data_after_comma(str_buf_ptr,1);
	str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
	str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
		if(GLOBAL_SSCANF((char *)str_buf, "%u", &tmp_int) == 1)
		{
			p_gnss_handle->gps_weeks = tmp_int;
		}
	}
	/* ȡ������ */
	str_buf_ptr = _gnss_nmea_get_data_after_comma(str_buf_ptr,1);
    str_cnt = _gnss_nmea_get_data_before_comma(str_buf_ptr, str_buf, MAX_TMP_STR_BUF_LEN);
    str_buf[str_cnt] = '\0';
    if (str_cnt)
    {
    	if((GLOBAL_SSCANF((char *)str_buf, "%u%c", &tmp_int, &tmp_char) == 2) 
			&& (tmp_char == 'D'))
    	{
    		p_gnss_handle->gps_leap_seconds = tmp_int;
    		APP_CLEAR_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_LEAP_FLAG);
    	}
		else if(GLOBAL_SSCANF((char *)str_buf, "%u", &tmp_int) == 1)
		{
			APP_SET_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_LEAP_FLAG);
			p_gnss_handle->gps_leap_seconds = tmp_int;
		}
    }
    else
    {
		p_gnss_handle->hdop = 99.99;
    }	

}


/*****************************************************************************
 �� �� ��  : ext_ref_ublox_parse_nmea
 ��������  : ����ublox NMEA����
 �������  : IN GNSS_REF_UBLOX_MSG_T cmd  
             IN U8 *data_ptr             
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2018��9��18�� ���ڶ�
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
static U8 _ext_ref_parse_nmea(IN GNSS_NMEA_MSG_ENUM cmd, IN U8 *data_ptr, IN U32 len)
{
	//GLOBAL_PRINT(("Entered ext_ref_parse_nmea !!\r\n"));
	switch(cmd)
	{
		case GNSS_NMEA_MSG_GGA:
			{
				/*
	   			*	Cmd Format:$GPGGA,074112.00,3958.7254293,N,11618.1966307,E,1,07,1.97,107.0023,M,-6.5229,M,,*4D
	   			*/
	   			//GLOBAL_PRINT(("Entered MSG_GGA !!\r\n"));
				_gnss_parse_gga(data_ptr,len);
			}
		break;

		case GNSS_NMEA_MSG_ZDA:
			{
				/*
	   			*	$GNZDA,005624.00,08,11,2012,00,00*74
	   			*/
				_gnss_parse_zda(data_ptr);
			}
			break;

		case GNSS_NMEA_MSG_RMC:
			{
				/*
	   			*	$GNZDA,005624.00,08,11,2012,00,00*74
	   			*/
				_gnss_parse_rmc(data_ptr);
			}
			break;
		case GNSS_NMEA_MSG_GSV:
			{
				/*
	   			*
	   			*/
				_gnss_parse_gsv(data_ptr);
			}
			break;

		case GNSS_NMEA_MSG_GSA:
			{
				/*
	   			*
	   			*/
				_gnss_parse_gsa(data_ptr);
			}
			break;

		case GNSS_NMEA_MSG_PUBX:
			{
				/*
	   			*
	   			*/
				_gnss_ublox_parse_pubx(data_ptr);
				//pubx_recv_flag = TRUE;
			}
			break;
		default:
		break;
	}

	return TRUE;
}

/*****************************************************************************
 �� �� ��  : gnss_ref_ublox_nmea_data_process
 ��������  : ����ublox���ջ��ı���
 �������  : IN U8 *data_ptr  
             IN U32 len       
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2018��9��18�� ���ڶ�
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
U8 gnss_nmea_data_process(IN U8 *data_ptr, IN U32 len)
{
	int i = 0;
	U8 cmd_found = FALSE;
    #define MAX_HEADER_LEN	9
	U8 tmp_buf[MAX_HEADER_LEN + 1];

	/* Calc XOR checksum */
	if (!_nmea_data_checksum(data_ptr, len))
	{
		return FALSE;
	}

	/* Copy receive machine header:
		$GNZDA,005624.00,08,11,2012,00,00*74,
		ǰ�����ַ��а�����ǰʹ��������Ϣ�� GNSS_REF_UBLOX_RCV_TYPE_T
	*/
	GLOBAL_STRNCPY((char *)tmp_buf, (char *)data_ptr, 3);
	tmp_buf[3] = '\0';


	/* Copy msg header */
	cmd_found = FALSE;
	GLOBAL_STRNCPY((char *)tmp_buf, (char *)data_ptr, (MAX_HEADER_LEN - 1));
	tmp_buf[MAX_HEADER_LEN] = '\0';

	/* Get msg type */
	for (i = GNSS_NMEA_MSG_GGA; i < GNSS_NMEA_MSG_END; i++)
	{
		if (GLOBAL_STRSTR((char *)tmp_buf, gnss_nmea_msg_map[i].msg_header))
		{
			cmd_found = TRUE;
			break;
		}
	}


	if (!cmd_found)
	{
		return FALSE;
	}

	/* Parse CMD */
	_ext_ref_parse_nmea((GNSS_NMEA_MSG_ENUM)i, data_ptr, len);

//
	char *pt = encode_Json();
	//GLOBAL_PRINT(("UDP Json: %s\r\n",pt));
	GLOBAL_FREE(pt);
	return TRUE;
}

/*****************************************************************************
 �� �� ��  : gnss_nmea_init
 ��������  : NMEA��ʼ��
 �������  : gnss_data_t* p_handle  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��5��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void gnss_nmea_init(gnss_data_t* p_handle)
{
	if(!p_handle)
	{
		ERR_PRINT(("ָ��Ƿ�!!!!!\r\n"));
		return;
	}
	p_gnss_handle = p_handle;
	GLOBAL_MEMSET(&NMEA_Data_ptr,0x0,sizeof(Parse_data));
}

/*eof*/
