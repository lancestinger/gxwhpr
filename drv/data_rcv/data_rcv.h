
#ifndef _DATA_RCV_H_
#define _DATA_RCV_H_

#include "pubdef.h"

typedef enum
{
	DATA_RCV_NORMAL,
	DATA_RCV_HEADER_1_OK,
	DATA_RCV_HEADER_2_OK,
	DATA_RCV_HEADER_3_OK,
	DATA_RCV_NEXT_HEADER_1_OK,

	DATA_RCV_TAIL_1_OK,
	DATA_RCV_TAIL_2_OK,
}UTL_DATA_RCV_STATUS_T;


typedef BOOL(* DATA_RCV_HANDLE_CB)(IN U8 *data_ptr, IN U32 data_len);

typedef struct
{
	U8 *data_buf_ptr;
	U32 current_len;
	BOOL in_use;
	BOOL buf_rcved;
}DATA_BUF_INFO_T;

typedef struct
{
	DATA_BUF_INFO_T *buf_info;
	U32 max_size;
	S32 current_buf_info_index;

	UTL_DATA_RCV_STATUS_T rcv_status;
	DATA_RCV_HANDLE_CB handle;

	U8 header1;
	U8 header2;
	U8 header3;
	U8 tail1;
	U8 tail2;
}UTL_DATA_RCV_T;

typedef struct
{
	char 	inuse;	//����ʹ�ñ�־
	char 	finishFlag;	//�������
	char* pbuf;		//����ָ��
	int bufLen;		//�ѻ������
}DATA_RCV_BUF_t;

typedef struct
{
	int rcv_buf_max_num;	//��󻺴����
	int rcv_buf_max_len;	//ÿ����������ֽ���
	int	currentLen;			//��ǰ���泤��
//	char finishFlag;		//�������
	char finishBufIndex;			//����ɻ����������
	char* p_currentBuf;		//��ǰ����ָ��
	DATA_RCV_BUF_t* p_rcv_s;	//���սṹ��ָ��
}DATA_RCV_Attr_t;

BOOL UTL_DATA_RCV_Data_For_1Head2Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data);
BOOL UTL_DATA_RCV_Data_For_1Head0Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data);
BOOL UTL_DATA_RCV_Data_For_0Head2Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data);
BOOL UTL_DATA_RCV_Data_For_2Head0Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data);	//for ubxЭ��
BOOL RTCM_DATA_RCV_Data_For_2Head0Tail(IN UTL_DATA_RCV_T *rcv, IN U8 data); //for RTCMЭ��


BOOL Data_DRV_CheckSum(IN U8 *data_ptr, IN U32 len);
U8 UTL_XOR(U8 *data, U32 len);
U8 UTL_HexToI(U8 str);

void rcvDataInit(void);
BOOL rcvDataRegister(DATA_RCV_BUF_t* rcv_s, const U8 buf_num, const int buf_len);
BOOL rcvDataInsert(DATA_RCV_BUF_t* rcv_s, U8* pData, U32 len);
void rcvDataFinish(DATA_RCV_BUF_t* rcv_s);
int rcvDataFlush(DATA_RCV_BUF_t* rcv_s, char* buf);
#endif
/*EOF*/
