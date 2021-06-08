/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : uart_api.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��5��29��
  ����޸�   :
  ��������   : ����FML����ģ�����
  �����б�   :
              print_def
              _print_thread
              uart_fetch_sshell_buf
              uart_fml_init
              _uart_rcv_sshell_data
              uart_set_dbg_outmode
  �޸���ʷ   :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/




/*------------------------------ͷ�ļ�------------------------------*/
#include "uart/uart_fml.h"
#include "rngbuf/rngbuf.h"
#include "comm/project_def.h"
/*------------------------------ͷ�ļ�------------------------------*/


/*------------------------------�ļ���------------------------------*/
#define PRINT_BUFF_LEN		SIZE_1K
/*------------------------------�ļ���------------------------------*/


/*------------------------------�ļ�����------------------------------*/
U8 printf_type = 1;		// 0��ʾֱ�ӼĴ�����ӡ	1��ʾͨ�������ӡ
static U8 dbg_out_mod = DBG_OUT_USART;
static CHAR_T txBuf[PRINT_BUFF_LEN];	//��ӡ��ʽ������
static U8 print_tmpbuf[PRINT_BUFF_LEN];	//��ӡ�������
RNGBUF_t * print_rngbuf = NULL;	//��ӡ�������
//static U8 receiver_tmpbuf[PRINT_BUFF_LEN]; //���ڽ��ջ�����
RNGBUF_t * receiver_rngbuf = NULL;  //���ڽ��ջ�����


static RNGBUF_t * sshell_rngbuf = NULL;	//shell���ջ���
static U64 thread_print_stk[SIZE_1K/ 8];
static const osThreadAttr_t thread_print_attr = {
  .stack_mem  = &thread_print_stk[0],
  .stack_size = sizeof(thread_print_stk),
  .priority = osPriorityBelowNormal,
};
/*------------------------------�ļ�����------------------------------*/


/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/




/*****************************************************************************
 �� �� ��  : print_def
 ��������  : �Զ����ӡ����
 �������  : CHAR_T * fmt  
             ...           
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void print_def(CHAR_T * fmt ,...)
{
	CHAR_T*  pTxd = txBuf;
	
  	va_list args;
	GLOBAL_MEMSET(pTxd,0,PRINT_BUFF_LEN);
	va_start(args, fmt);
	vsprintf(pTxd,fmt,args);
	va_end(args);

	if((printf_type) && print_rngbuf)
	{
		if(rng_get_free_bytes(print_rngbuf) >= GLOBAL_STRLEN(pTxd))
		{
			rng_put_buf(print_rngbuf, pTxd, GLOBAL_STRLEN(pTxd));
		}
	}
	else
	{
		uart_send_data(DEBUG_PRINT_COM, (U8*)pTxd, GLOBAL_STRLEN(pTxd));
	}

}

/*****************************************************************************
 �� �� ��  : uart_set_dbg_outmode
 ��������  : ���ô�ӡ���ģʽ
 �������  : U8 mode  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void uart_set_dbg_outmode(U8 mode)
{
	dbg_out_mod = mode;
}

/*****************************************************************************
 �� �� ��  : uart_get_dbg_outmode
 ��������  : ��ȡ��ӡ���ģʽ
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
U8 uart_get_dbg_outmode(void)
{
	return dbg_out_mod;
}

/*****************************************************************************
 �� �� ��  : _uart_rcv_sshell_data
 ��������  : shell�������ݻ���
 �������  : U8 data  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
static BOOL _uart_rcv_sshell_data(CHAR_T data)
{
	uart_set_dbg_outmode(DBG_OUT_USART);
	rng_put_buf(sshell_rngbuf, &data, 1);
	
	return 0;
}

/*****************************************************************************
 �� �� ��  : uart_fetch_sshell_buf
 ��������  : ȡsshell���ݻ���
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
CHAR_T* uart_fetch_sshell_buf(void)
{
	static CHAR_T data = 0;

	if(rng_get_buf(sshell_rngbuf, &data, 1) > 0)
	{
		return &data;
	}
	else
	{
		return NULL;
	}
}

/*****************************************************************************
 �� �� ��  : _print_thread
 ��������  : ��ӡ�̣߳����ȼ�ΪLOW
 �������  : void *arg  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��30��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
static void _print_thread(void *arg)
{
	S32 len = 0;
	
	while(1)
	{
		if((dbg_out_mod == DBG_OUT_USART) && ((len = rng_get_buf(print_rngbuf, (char*)print_tmpbuf, PRINT_BUFF_LEN)) > 0))
		{
			uart_send_data(DEBUG_PRINT_COM, print_tmpbuf, len);
		}
        // else if((g_dbg_out_usart_model==DBG_OUT_USART_RECEIVER_MODEL)
        //             &&((len = rng_get_buf(receiver_rngbuf, (char*)receiver_tmpbuf, PRINT_BUFF_LEN))>0))
        // {
        //     uart_send_data(DEBUG_PRINT_COM, receiver_tmpbuf, len);
        // }
		else
		{
			delay_ms(2);
		}
		//osThreadYield();
	}
}

/*****************************************************************************
 �� �� ��  : uart_fml_init
 ��������  : ���ڹ���ģ���ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��5��29��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void uart_fml_init(void)
{
	osThreadId_t thread_print_id = 0;
	
	print_rngbuf = rng_create(SIZE_8K);	// 8KB����
	sshell_rngbuf = rng_create(SIZE_256);
    receiver_rngbuf = rng_create(SIZE_1K);
    if(!receiver_rngbuf)
        ERR_PRINT(("receiver_rngbuf create failed!!!\r\n"));
	uart_register_rx_callbak(DEBUG_PRINT_COM, _uart_rcv_sshell_data);	//ע����Դ���
	//uart_register_rx_callbak(DEBUG_PRINT_COM_BAK, _uart_rcv_sshell_data);	//ע����Դ���
	thread_print_id = osThreadNew(_print_thread, NULL, &thread_print_attr);
	GLOBAL_HEX(thread_print_id);
}

/*eof*/
