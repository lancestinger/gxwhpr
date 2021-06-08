/******************************************************************************

                  版权所有 (C), 2018-2020, GridSatellite

 ******************************************************************************
  文 件 名   : data_rcv.c
  版 本 号   : 初稿
  作    者   : wenzhiquan
  生成日期   : 2018年3月26日 星期一
  最近修改   :
  功能描述   : 数据缓存模块
  函数列表   :
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
  修改历史   :
  1.日    期   : 2018年3月26日 星期一
    作    者   : wenzhiquan
    修改内容   : 创建文件

******************************************************************************/


#include "fml/gnss/nmea.h"
#include "data_rcv.h"

/*****************************************************************************
 函 数 名  : UTL_DATA_RCV_Data_For_1Head2Tail
 功能描述  : 数据接收函数（一个帧头两个帧尾）
 输入参数  : IN UTL_DATA_RCV_T *rcv  
             IN U8 data           
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年11月26日
    作    者   : Winlab
    修改内容   : 新生成函数

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
				UBX_1PPS_time = osKernelGetTickCount();//暂时用于惯导1PPS时间戳获取
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
 函 数 名  : UTL_DATA_RCV_Data_For_1Head0Tail
 功能描述  : 缓存一个帧头无帧尾的数据包，比较特殊
 输入参数  : IN UTL_DATA_RCV_T *rcv  
             IN U8 data              
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年10月10日
    作    者   : wenzhiquan
    修改内容   : 新生成函数

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
 函 数 名  : UTL_DATA_RCV_Data_For_2Head0Tail
 功能描述  : 缓存两个帧头无帧尾的数据包,适用于UBX协议
 输入参数  : IN UTL_DATA_RCV_T *rcv  
             IN U8 data              
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年12月7日 星期五
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
BOOL UTL_DATA_RCV_Data_For_2Head0Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data)
{
	
	if ((rcv->current_buf_info_index == -1) || (!rcv))
	{
		/* Drop this data, Error buffer info index, mybe no aviable buffer exits */
		//ERR_PRINT(("接收缓存无效!!!!\r\n"));
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
				if(rcv->buf_info->current_len >= 6)	//接收完长度字节
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
 函 数 名  : RTCM_DATA_RCV_Data_For_2Head0Tail
 功能描述  : 缓存两个帧头无帧尾的数据包,适用于RTCM协议
 输入参数  : IN UTL_DATA_RCV_T *rcv  
             IN U8 data              
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月2日 星期二
    作    者   : zxf
    修改内容   : 新生成函数

*****************************************************************************/
BOOL RTCM_DATA_RCV_Data_For_2Head0Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data)
{
	
	if ((rcv->current_buf_info_index == -1) || (!rcv))
	{
		/* Drop this data, Error buffer info index, mybe no aviable buffer exits */
		//ERR_PRINT(("接收缓存无效!!!!\r\n"));
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
				if(rcv->buf_info->current_len >= 3)	//接收完长度字节
				{
					U16 frame_len = rcv->buf_info->data_buf_ptr[2] + 6;//RTCM帧头+帧尾为 6字节

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
	*					$AACCC，c—c*hh<CR><LF>
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

#define DATA_RCV_NUM 2		//需要接收的不同接口来源的数量,比如从2个串口分别接收数据，就填2

static DATA_RCV_Attr_t m_dataRcv_s[DATA_RCV_NUM];

/*****************************************************************************
 函 数 名  : rcvDataInit
 功能描述  : 初始化数据接收结构体
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年11月17日
    作    者   : wenzhiquan
    修改内容   : 新生成函数

*****************************************************************************/
void rcvDataInit(void)
{
	memset(m_dataRcv_s, 0x0, sizeof(m_dataRcv_s));
}

/*****************************************************************************
 函 数 名  : rcvDataRegister
 功能描述  : 注册接收数据结构
 输入参数  : DATA_RCV_BUF_t* rcv_s  
             U8 buf_num    建议>= 2         
             int buf_len            
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年11月15日
    作    者   : wenzhiquan
    修改内容   : 新生成函数

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
		if(m_dataRcv_s[i].p_rcv_s == NULL)		//为空才能注册
		{
			m_dataRcv_s[i].p_rcv_s = rcv_s;
			m_dataRcv_s[i].rcv_buf_max_num = buf_num;
			m_dataRcv_s[i].rcv_buf_max_len = buf_len;
			break;
		}
	}
	if(i == DATA_RCV_NUM)
	{
		ERR_PRINT(("m_dataRcv_s 空间不足!!!\r\n"));
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
 函 数 名  : rcvDataInsert
 功能描述  : 向缓存插入数据
 输入参数  : DATA_RCV_BUF_t* rcv_s  
             U8* pData     
             U32 len
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年11月15日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
BOOL rcvDataInsert(DATA_RCV_BUF_t* rcv_s, U8* pData, U32 len)
{
	U8 i = 0;

	if(!rcv_s)
	{
		ERR_PRINT(("rcv_s指针非法!!!!\r\n"));
		return FALSE;
	}
	for(i = 0; i < DATA_RCV_NUM; i++)
	{
		if(rcv_s == m_dataRcv_s[i].p_rcv_s)
		{
			if((m_dataRcv_s[i].currentLen >= m_dataRcv_s[i].rcv_buf_max_len) ||
				((m_dataRcv_s[i].currentLen + len) > m_dataRcv_s[i].rcv_buf_max_len))
			{
				ERR_PRINT(("缓存溢出!!!!\r\n"));
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
 函 数 名  : rcvDataFinish
 功能描述  : 缓存一帧完整数据需要置标志位，且需要准备下一帧地址
 输入参数  : DATA_RCV_BUF_t* rcv_s  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年11月15日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
void rcvDataFinish(DATA_RCV_BUF_t* rcv_s)
{
	U8 i = 0, j  = 0, k = 0;

	if(!rcv_s)
	{
		ERR_PRINT(("rcv_s指针非法!!!!\r\n"));
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
 函 数 名  : rcvDataFlush
 功能描述  : 取数据，清缓存
 输入参数  : DATA_RCV_BUF_t* rcv_s  
             char* buf              
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年11月15日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
int rcvDataFlush(DATA_RCV_BUF_t* rcv_s, char* buf)
{
	U8 i = 0, j = 0, k = 0;
	int rlen = 0;
	static U8 preFinishItem[DATA_RCV_NUM] = {0};

	if(!rcv_s)
	{
		ERR_PRINT(("rcv_s指针非法!!!!\r\n"));
		return 0;
	}
	for(i = 0; i < DATA_RCV_NUM; i++)
	{
		if(rcv_s == m_dataRcv_s[i].p_rcv_s)
		{
			for(k = 0; k < m_dataRcv_s[i].rcv_buf_max_num; k++)
			{
				j = (preFinishItem[i] + k + 1) % (m_dataRcv_s[i].rcv_buf_max_num);
				if(rcv_s[j].finishFlag == TRUE)	//存在接收完整帧
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
