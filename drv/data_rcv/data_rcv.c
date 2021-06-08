/******************************************************************************

                  ��Ȩ���� (C), 2018-2020, GridSatellite

 ******************************************************************************
  �� �� ��   : data_rcv.c
  �� �� ��   : ����
  ��    ��   : wenzhiquan
  ��������   : 2018��3��26�� ����һ
  ����޸�   :
  ��������   : ���ݻ���ģ��
  �����б�   :
              Data_DRV_CheckSum
              rcvDataFinish
              rcvDataFlush
              rcvDataInit
              rcvDataInsert
              rcvDataRegister
              UTL_DATA_RCV_Data_For_0Head2Tail
              UTL_DATA_RCV_Data_For_1Head0Tail
              UTL_DATA_RCV_Data_For_1Head2Tail
              UTL_HexToI
              UTL_XOR
  �޸���ʷ   :
  1.��    ��   : 2018��3��26�� ����һ
    ��    ��   : wenzhiquan
    �޸�����   : �����ļ�

******************************************************************************/


#include "fml/gnss/nmea.h"
#include "data_rcv.h"

/*****************************************************************************
 �� �� ��  : UTL_DATA_RCV_Data_For_1Head2Tail
 ��������  : ���ݽ��պ�����һ��֡ͷ����֡β��
 �������  : IN UTL_DATA_RCV_T *rcv  
             IN U8 data           
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��11��26��
    ��    ��   : Winlab
    �޸�����   : �����ɺ���

*****************************************************************************/
BOOL UTL_DATA_RCV_Data_For_1Head2Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data)
{
	
	if ((rcv->current_buf_info_index == -1) || (!rcv))
	{
		/* Drop this data, Error buffer info index, mybe no aviable buffer exits */
		return FALSE;
	}

	if (!rcv->buf_info->in_use)
	{
		rcv->buf_info->in_use = TRUE;
	}

	if (rcv->buf_info->current_len == 0)
	{
		if (data != rcv->header1)
		{
			/* Drop this data, Error Header */
			return FALSE;
		}
	}
	else if (rcv->buf_info->current_len > (rcv->max_size - 2))
	{
		/* Error Data */
		rcv->buf_info->current_len = 0;
		rcv->rcv_status = DATA_RCV_NORMAL;

		return FALSE;
	}

	switch (rcv->rcv_status)
	{
		case DATA_RCV_NORMAL:
			if (rcv->header1 != data)
			{
				rcv->rcv_status = DATA_RCV_NORMAL;
				rcv->buf_info->current_len = 0;

				return FALSE;
			}
			else
			{
				/* Header 1 OK */
				UBX_1PPS_time = osKernelGetTickCount();//��ʱ���ڹߵ�1PPSʱ�����ȡ
				rcv->rcv_status = DATA_RCV_HEADER_1_OK;
			}
		break;

		case DATA_RCV_HEADER_1_OK:
			{
				if (data == rcv->header1)
				{
					/* Drop frontal data */
					rcv->rcv_status = DATA_RCV_HEADER_1_OK;
					memset (rcv->buf_info->data_buf_ptr, 0x0, rcv->max_size -2);
                    rcv->buf_info->data_buf_ptr[0] = rcv->header1;
                    rcv->buf_info->current_len = 1;
                    return FALSE;
				}
				else if (data == rcv->tail1)
				{
					rcv->rcv_status = DATA_RCV_TAIL_1_OK;
				}
				else
				{
					/* Normal Data, Not special Header or tail */
					break;
				}
			}
		break;

		case DATA_RCV_TAIL_1_OK:
			if (rcv->header1 == data)
			{
				/* tail 1 ok, but a new header1 coming, drop previous data */
				rcv->rcv_status = DATA_RCV_HEADER_1_OK;
				memset (rcv->buf_info->data_buf_ptr, 0x0, rcv->max_size -2);
                rcv->buf_info->data_buf_ptr[0] = rcv->header1;
                rcv->buf_info->current_len = 1;

				return FALSE;
			}
			else if (rcv->tail2 != data)
			{
				/* Sorry, Previous Data not tail, Go to receive Valid tail */
				rcv->rcv_status = DATA_RCV_HEADER_1_OK;
			}
			else
			{
				/* A valid data coming */
				rcv->rcv_status = DATA_RCV_TAIL_2_OK;
			}
		break;
	}

	/* Record Data */
	*(rcv->buf_info->data_buf_ptr + rcv->buf_info->current_len) = data;
	rcv->buf_info->current_len++;

	if (rcv->rcv_status == DATA_RCV_TAIL_2_OK)
	{
		rcv->buf_info->buf_rcved = TRUE;
		(*rcv->handle)(rcv->buf_info->data_buf_ptr, rcv->buf_info->current_len);

		/* Reset Receive status */
		rcv->rcv_status = DATA_RCV_NORMAL;
	}

	return TRUE;
}

/*****************************************************************************
 �� �� ��  : UTL_DATA_RCV_Data_For_1Head0Tail
 ��������  : ����һ��֡ͷ��֡β�����ݰ����Ƚ�����
 �������  : IN UTL_DATA_RCV_T *rcv  
             IN U8 data              
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2017��10��10��
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
BOOL UTL_DATA_RCV_Data_For_1Head0Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data)
{
	
	if ((rcv->current_buf_info_index == -1) || (!rcv))
	{
		/* Drop this data, Error buffer info index, mybe no aviable buffer exits */
		return FALSE;
	}

	if (!rcv->buf_info->in_use)
	{
		rcv->buf_info->in_use = TRUE;
	}

	if (rcv->buf_info->current_len == 0)
	{
		if (data != rcv->header1)
		{
			/* Drop this data, Error Header */
			return FALSE;
		}
	}
	else if (rcv->buf_info->current_len > (rcv->max_size - 2))
	{
		/* Error Data */
		rcv->buf_info->current_len = 0;
		rcv->rcv_status = DATA_RCV_NORMAL;

		return FALSE;
	}

	switch (rcv->rcv_status)
	{
		case DATA_RCV_NORMAL:
			if (rcv->header1 != data)
			{
				rcv->rcv_status = DATA_RCV_NORMAL;
				rcv->buf_info->current_len = 0;

				return FALSE;
			}
			else
			{
				/* Header 1 OK */
				rcv->rcv_status = DATA_RCV_HEADER_1_OK;
			}
		break;

		case DATA_RCV_HEADER_1_OK:
			{
				if (data == rcv->header1)
				{
					/* Drop frontal data */
					rcv->rcv_status = DATA_RCV_HEADER_1_OK;
					memset (rcv->buf_info->data_buf_ptr, 0x0, rcv->max_size -2);
                    rcv->buf_info->data_buf_ptr[0] = rcv->header1;
                    rcv->buf_info->current_len = 1;
                    return FALSE;
				}
				else if (data == rcv->tail1)
				{
					rcv->rcv_status = DATA_RCV_TAIL_1_OK;
				}
				else
				{
					/* Normal Data, Not special Header or tail */
					break;
				}
			}
		break;

		case DATA_RCV_TAIL_1_OK:
			if (rcv->header1 == data)
			{
				/* tail 1 ok, but a new header1 coming, drop previous data */
				rcv->rcv_status = DATA_RCV_HEADER_1_OK;
				memset (rcv->buf_info->data_buf_ptr, 0x0, rcv->max_size -2);
                rcv->buf_info->data_buf_ptr[0] = rcv->header1;
                rcv->buf_info->current_len = 1;

				return FALSE;
			}
			else if (rcv->tail2 != data)
			{
				/* Sorry, Previous Data not tail, Go to receive Valid tail */
				rcv->rcv_status = DATA_RCV_HEADER_1_OK;
			}
			else
			{
				/* A valid data coming */
				rcv->rcv_status = DATA_RCV_TAIL_2_OK;
			}
		break;
	}

	/* Record Data */
	*(rcv->buf_info->data_buf_ptr + rcv->buf_info->current_len++) = data;

	if (rcv->rcv_status == DATA_RCV_TAIL_2_OK)
	{
		rcv->buf_info->buf_rcved = TRUE;
		(*rcv->handle)(rcv->buf_info->data_buf_ptr, rcv->buf_info->current_len);

		/* Reset Receive status */
		rcv->rcv_status = DATA_RCV_NORMAL;
	}

	return TRUE;
}


/*****************************************************************************
 �� �� ��  : UTL_DATA_RCV_Data_For_2Head0Tail
 ��������  : ��������֡ͷ��֡β�����ݰ�,������UBXЭ��
 �������  : IN UTL_DATA_RCV_T *rcv  
             IN U8 data              
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2018��12��7�� ������
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
BOOL UTL_DATA_RCV_Data_For_2Head0Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data)
{
	
	if ((rcv->current_buf_info_index == -1) || (!rcv))
	{
		/* Drop this data, Error buffer info index, mybe no aviable buffer exits */
		//ERR_PRINT(("���ջ�����Ч!!!!\r\n"));
		return FALSE;
	}

	if (!rcv->buf_info->in_use)
	{
		rcv->buf_info->in_use = TRUE;
	}

	if (rcv->buf_info->current_len == 0)
	{
		if (data != rcv->header1)
		{
			/* Drop this data, Error Header */
			return FALSE;
		}
	}
	else if (rcv->buf_info->current_len > (rcv->max_size - 2))
	{
		/* Error Data */
		rcv->buf_info->current_len = 0;
		rcv->rcv_status = DATA_RCV_NORMAL;

		return FALSE;
	}

	switch (rcv->rcv_status)
	{
		case DATA_RCV_NORMAL:
			if (rcv->header1 != data)
			{
				rcv->rcv_status = DATA_RCV_NORMAL;
				rcv->buf_info->current_len = 0;

				return FALSE;
			}
			else
			{
				/* Header 1 OK */
				rcv->rcv_status = DATA_RCV_HEADER_1_OK;
			}
		break;

		case DATA_RCV_HEADER_1_OK:
			if (rcv->header2 != data)
			{
				rcv->rcv_status = DATA_RCV_NORMAL;
				rcv->buf_info->current_len = 0;

				return FALSE;
			}
			else
			{
				/* Header 1 OK */
				rcv->rcv_status = DATA_RCV_HEADER_2_OK;
			}
		break;

		case DATA_RCV_HEADER_2_OK:
			{
				if(rcv->buf_info->current_len >= 6)	//�����곤���ֽ�
				{
					U16 frame_len = rcv->buf_info->data_buf_ptr[4] + rcv->buf_info->data_buf_ptr[5]*256 + 8;

					if((rcv->buf_info->current_len+1) >= frame_len)
					{
						rcv->rcv_status = DATA_RCV_TAIL_2_OK;
					}
				}
			}
		break;

		default:
			break;
	}

	/* Record Data */
	*(rcv->buf_info->data_buf_ptr + rcv->buf_info->current_len) = data;
	rcv->buf_info->current_len++;

	if (rcv->rcv_status == DATA_RCV_TAIL_2_OK)
	{
		rcv->buf_info->buf_rcved = TRUE;
		(*rcv->handle)(rcv->buf_info->data_buf_ptr, rcv->buf_info->current_len);

		/* Reset Receive status */
		rcv->rcv_status = DATA_RCV_NORMAL;
	}

	return TRUE;
}

/*****************************************************************************
 �� �� ��  : RTCM_DATA_RCV_Data_For_2Head0Tail
 ��������  : ��������֡ͷ��֡β�����ݰ�,������RTCMЭ��
 �������  : IN UTL_DATA_RCV_T *rcv  
             IN U8 data              
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��2�� ���ڶ�
    ��    ��   : zxf
    �޸�����   : �����ɺ���

*****************************************************************************/
BOOL RTCM_DATA_RCV_Data_For_2Head0Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data)
{
	
	if ((rcv->current_buf_info_index == -1) || (!rcv))
	{
		/* Drop this data, Error buffer info index, mybe no aviable buffer exits */
		//ERR_PRINT(("���ջ�����Ч!!!!\r\n"));
		return FALSE;
	}

	if (!rcv->buf_info->in_use)
	{
		rcv->buf_info->in_use = TRUE;
	}

	if (rcv->buf_info->current_len == 0)
	{
		if (data != rcv->header1)
		{
			/* Drop this data, Error Header */
			GLOBAL_PRINT(("\r\nHead 1 FAILED!!!!\r\n"));
			return FALSE;
		}
	}
	else if (rcv->buf_info->current_len > (rcv->max_size - 2))
	{
		/* Error Data */
		rcv->buf_info->current_len = 0;
		rcv->rcv_status = DATA_RCV_NORMAL;
		GLOBAL_PRINT(("\r\nHead 1 SUCCESS!!!!\r\n"));

		return FALSE;
	}

	switch (rcv->rcv_status)
	{
		case DATA_RCV_NORMAL:
			if (rcv->header1 != data)
			{
				rcv->rcv_status = DATA_RCV_NORMAL;
				rcv->buf_info->current_len = 0;

				return FALSE;
			}
			else
			{
				/* Header 1 OK */
				GLOBAL_PRINT(("\r\nHead 1 OK!!!!\r\n"));
				rcv->rcv_status = DATA_RCV_HEADER_1_OK;
			}
		break;

		case DATA_RCV_HEADER_1_OK:
			if (rcv->header2 != data)
			{
				rcv->rcv_status = DATA_RCV_NORMAL;
				rcv->buf_info->current_len = 0;

				return FALSE;
			}
			else
			{
				/* Header 2 OK */
				GLOBAL_PRINT(("\r\nHead 2 OK!!!!\r\n"));
				rcv->rcv_status = DATA_RCV_HEADER_2_OK;
			}
		break;

		case DATA_RCV_HEADER_2_OK:
			{
				if(rcv->buf_info->current_len >= 3)	//�����곤���ֽ�
				{
					U16 frame_len = rcv->buf_info->data_buf_ptr[2] + 6;//RTCM֡ͷ+֡βΪ 6�ֽ�

					GLOBAL_PRINT(("\r\nFrame_len = %d",frame_len));

					if((rcv->buf_info->current_len+1) >= frame_len)
					{
						rcv->rcv_status = DATA_RCV_TAIL_2_OK;
					}
				}
			}
		break;

		default:
			break;
	}

	/* Record Data */
	*(rcv->buf_info->data_buf_ptr + rcv->buf_info->current_len) = data;
	rcv->buf_info->current_len++;

	if (rcv->rcv_status == DATA_RCV_TAIL_2_OK)
	{
		rcv->buf_info->buf_rcved = TRUE;
		(*rcv->handle)(rcv->buf_info->data_buf_ptr, rcv->buf_info->current_len);

		/* Reset Receive status */
		rcv->rcv_status = DATA_RCV_NORMAL;
	}

	return TRUE;
}



BOOL UTL_DATA_RCV_Data_For_0Head2Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data)
{
	if ((rcv->current_buf_info_index == -1) || (!rcv))
	{
		/* Drop this data, Error buffer info index, mybe no aviable buffer exits */
		return FALSE;
	}

	if (!rcv->buf_info->in_use)
	{
		rcv->buf_info->in_use = TRUE;
	}

	if (rcv->buf_info->current_len == 0)
	{
		rcv->rcv_status = DATA_RCV_NORMAL;
	}
	else if (rcv->buf_info->current_len > (rcv->max_size - 2))
	{
		/* Error Data */
		rcv->buf_info->current_len = 0;
		rcv->rcv_status = DATA_RCV_NORMAL;

		return FALSE;
	}

	switch (rcv->rcv_status)
	{
		case DATA_RCV_NORMAL:
			if (data == rcv->tail1)
			{
				rcv->rcv_status = DATA_RCV_TAIL_1_OK;
			}
			else
			{
				/* Normal Data, Not special Header or tail */
				break;
			}
		break;

		case DATA_RCV_TAIL_1_OK:
			if (rcv->tail2 != data)
			{
				/* Sorry, Previous Data not tail, Go to receive Valid tail */
				rcv->rcv_status = DATA_RCV_NORMAL;
			}
			else
			{
				/* A valid data coming */
				rcv->rcv_status = DATA_RCV_TAIL_2_OK;
			}
		break;
	}

	/* Record Data */
	*(rcv->buf_info->data_buf_ptr + rcv->buf_info->current_len++) = data;

	if (rcv->rcv_status == DATA_RCV_TAIL_2_OK)
	{
		rcv->buf_info->buf_rcved = TRUE;
		(*rcv->handle)(rcv->buf_info->data_buf_ptr, rcv->buf_info->current_len);

		/* Reset Receive status */
		rcv->rcv_status = DATA_RCV_NORMAL;
	}

	return TRUE;
}

U8 UTL_HexToI(U8 str)
{
	U8			ch;
	unsigned int 	val = 0;

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

U8 UTL_XOR(U8 *data, U32 len)
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


BOOL Data_DRV_CheckSum(IN U8 *data_ptr, IN U32 len)
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

	xor_checksum_inmsg = UTL_HexToI(*(data_ptr + len - 4));
	xor_checksum_inmsg <<= 4;
	xor_checksum_inmsg += UTL_HexToI(*(data_ptr + len - 3));

	/* Calc Data check sum */
	xor_checksum = UTL_XOR(data_ptr + 1, len - 6);

	if (xor_checksum != xor_checksum_inmsg)
	{
		ERR_PRINT(("\r\n CKS: %02x, %02x", xor_checksum_inmsg, xor_checksum));

		return FALSE;
	}

	return TRUE;
}

#define DATA_RCV_NUM 2		//��Ҫ���յĲ�ͬ�ӿ���Դ������,�����2�����ڷֱ�������ݣ�����2

static DATA_RCV_Attr_t m_dataRcv_s[DATA_RCV_NUM];

/*****************************************************************************
 �� �� ��  : rcvDataInit
 ��������  : ��ʼ�����ݽ��սṹ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2017��11��17��
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
void rcvDataInit(void)
{
	memset(m_dataRcv_s, 0x0, sizeof(m_dataRcv_s));
}

/*****************************************************************************
 �� �� ��  : rcvDataRegister
 ��������  : ע��������ݽṹ
 �������  : DATA_RCV_BUF_t* rcv_s  
             U8 buf_num    ����>= 2         
             int buf_len            
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2017��11��15��
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
BOOL rcvDataRegister(DATA_RCV_BUF_t* rcv_s, const U8 buf_num, const int buf_len)
{
	U8 i = 0, index = 0;

	if(!rcv_s)
	{
		return FALSE;
	}
	for(i = 0; i < DATA_RCV_NUM; i++)
	{
		if(m_dataRcv_s[i].p_rcv_s == NULL)		//Ϊ�ղ���ע��
		{
			m_dataRcv_s[i].p_rcv_s = rcv_s;
			m_dataRcv_s[i].rcv_buf_max_num = buf_num;
			m_dataRcv_s[i].rcv_buf_max_len = buf_len;
			break;
		}
	}
	if(i == DATA_RCV_NUM)
	{
		ERR_PRINT(("m_dataRcv_s �ռ䲻��!!!\r\n"));
		return FALSE;
	}
	index = i;
	memset(rcv_s, 0x0, (sizeof(DATA_RCV_BUF_t)*buf_num));
	for(i = 0; i < buf_num; i++)
	{
		if((rcv_s[i].pbuf = (char *)malloc(buf_len)) == NULL)
		{
			return FALSE;
		}
	}
	m_dataRcv_s[index].finishBufIndex = 0;
	m_dataRcv_s[index].p_currentBuf = rcv_s[0].pbuf;
	rcv_s[0].inuse = TRUE;
	rcvDataFinish(rcv_s);
	return TRUE;
}

/*****************************************************************************
 �� �� ��  : rcvDataInsert
 ��������  : �򻺴��������
 �������  : DATA_RCV_BUF_t* rcv_s  
             U8* pData     
             U32 len
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2017��11��15��
    ��    ��   : Winlab
    �޸�����   : �����ɺ���

*****************************************************************************/
BOOL rcvDataInsert(DATA_RCV_BUF_t* rcv_s, U8* pData, U32 len)
{
	U8 i = 0;

	if(!rcv_s)
	{
		ERR_PRINT(("rcv_sָ��Ƿ�!!!!\r\n"));
		return FALSE;
	}
	for(i = 0; i < DATA_RCV_NUM; i++)
	{
		if(rcv_s == m_dataRcv_s[i].p_rcv_s)
		{
			if((m_dataRcv_s[i].currentLen >= m_dataRcv_s[i].rcv_buf_max_len) ||
				((m_dataRcv_s[i].currentLen + len) > m_dataRcv_s[i].rcv_buf_max_len))
			{
				ERR_PRINT(("�������!!!!\r\n"));
				return FALSE;
			}
			if(m_dataRcv_s[i].p_currentBuf)
			{
				memcpy(&m_dataRcv_s[i].p_currentBuf[m_dataRcv_s[i].currentLen], pData, len);
				m_dataRcv_s[i].currentLen = m_dataRcv_s[i].currentLen + len;
				//m_dataRcv_s[i].p_currentBuf[m_dataRcv_s[i].currentLen++] = rcvData;
			}
		}
	}
	
	return TRUE;
}

/*****************************************************************************
 �� �� ��  : rcvDataFinish
 ��������  : ����һ֡����������Ҫ�ñ�־λ������Ҫ׼����һ֡��ַ
 �������  : DATA_RCV_BUF_t* rcv_s  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2017��11��15��
    ��    ��   : Winlab
    �޸�����   : �����ɺ���

*****************************************************************************/
void rcvDataFinish(DATA_RCV_BUF_t* rcv_s)
{
	U8 i = 0, j  = 0, k = 0;

	if(!rcv_s)
	{
		ERR_PRINT(("rcv_sָ��Ƿ�!!!!\r\n"));
		return;
	}
	for(i = 0; i < DATA_RCV_NUM; i++)
	{
		if(rcv_s == m_dataRcv_s[i].p_rcv_s)
		{
			//m_dataRcv_s[i].finishFlag = TRUE;
			rcv_s[m_dataRcv_s[i].finishBufIndex].bufLen = m_dataRcv_s[i].currentLen;
			rcv_s[m_dataRcv_s[i].finishBufIndex].finishFlag = TRUE;
			for(k = 0; k < m_dataRcv_s[i].rcv_buf_max_num; k++)
			{
				j = (k + m_dataRcv_s[i].finishBufIndex + 1) % m_dataRcv_s[i].rcv_buf_max_num;
				if(rcv_s[j].inuse == FALSE)
				{
					rcv_s[j].inuse = TRUE;
					m_dataRcv_s[i].p_currentBuf = rcv_s[j].pbuf;
					m_dataRcv_s[i].currentLen = 0;
					m_dataRcv_s[i].finishBufIndex = j;
					//rcv_s[j].bufLen = 0;
					break;
				}
			}
			
		}
	}
}

/*****************************************************************************
 �� �� ��  : rcvDataFlush
 ��������  : ȡ���ݣ��建��
 �������  : DATA_RCV_BUF_t* rcv_s  
             char* buf              
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2017��11��15��
    ��    ��   : Winlab
    �޸�����   : �����ɺ���

*****************************************************************************/
int rcvDataFlush(DATA_RCV_BUF_t* rcv_s, char* buf)
{
	U8 i = 0, j = 0, k = 0;
	int rlen = 0;
	static U8 preFinishItem[DATA_RCV_NUM] = {0};

	if(!rcv_s)
	{
		ERR_PRINT(("rcv_sָ��Ƿ�!!!!\r\n"));
		return 0;
	}
	for(i = 0; i < DATA_RCV_NUM; i++)
	{
		if(rcv_s == m_dataRcv_s[i].p_rcv_s)
		{
			for(k = 0; k < m_dataRcv_s[i].rcv_buf_max_num; k++)
			{
				j = (preFinishItem[i] + k + 1) % (m_dataRcv_s[i].rcv_buf_max_num);
				if(rcv_s[j].finishFlag == TRUE)	//���ڽ�������֡
				{
					rlen = rcv_s[j].bufLen;
					memcpy(buf, rcv_s[j].pbuf, rlen);
					rcv_s[j].finishFlag = FALSE;
					rcv_s[j].inuse = FALSE;
					rcv_s[j].bufLen = 0;
					preFinishItem[i] = j;
					//GLOBAL_INTVAL(j);
					return rlen;
				}
			}
		}
	}

	return rlen;
}
