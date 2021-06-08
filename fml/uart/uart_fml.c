/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : uart_api.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年5月29日
  最近修改   :
  功能描述   : 串口FML功能模块程序
  函数列表   :
              print_def
              _print_thread
              uart_fetch_sshell_buf
              uart_fml_init
              _uart_rcv_sshell_data
              uart_set_dbg_outmode
  修改历史   :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/




/*------------------------------头文件------------------------------*/
#include "uart/uart_fml.h"
#include "rngbuf/rngbuf.h"
#include "comm/project_def.h"
/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/
#define PRINT_BUFF_LEN		SIZE_1K
/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
U8 printf_type = 1;		// 0表示直接寄存器打印	1表示通过缓存打印
static U8 dbg_out_mod = DBG_OUT_USART;
static CHAR_T txBuf[PRINT_BUFF_LEN];	//打印格式化缓存
static U8 print_tmpbuf[PRINT_BUFF_LEN];	//打印输出缓存
RNGBUF_t * print_rngbuf = NULL;	//打印输出队列
//static U8 receiver_tmpbuf[PRINT_BUFF_LEN]; //用于接收机调试
RNGBUF_t * receiver_rngbuf = NULL;  //用于接收机调试


static RNGBUF_t * sshell_rngbuf = NULL;	//shell接收缓存
static U64 thread_print_stk[SIZE_1K/ 8];
static const osThreadAttr_t thread_print_attr = {
  .stack_mem  = &thread_print_stk[0],
  .stack_size = sizeof(thread_print_stk),
  .priority = osPriorityBelowNormal,
};
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/




/*****************************************************************************
 函 数 名  : print_def
 功能描述  : 自定义打印函数
 输入参数  : CHAR_T * fmt  
             ...           
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 新生成函数

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
 函 数 名  : uart_set_dbg_outmode
 功能描述  : 设置打印输出模式
 输入参数  : U8 mode  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void uart_set_dbg_outmode(U8 mode)
{
	dbg_out_mod = mode;
}

/*****************************************************************************
 函 数 名  : uart_get_dbg_outmode
 功能描述  : 获取打印输出模式
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
U8 uart_get_dbg_outmode(void)
{
	return dbg_out_mod;
}

/*****************************************************************************
 函 数 名  : _uart_rcv_sshell_data
 功能描述  : shell接收数据缓存
 输入参数  : U8 data  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月29日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static BOOL _uart_rcv_sshell_data(CHAR_T data)
{
	uart_set_dbg_outmode(DBG_OUT_USART);
	rng_put_buf(sshell_rngbuf, &data, 1);
	
	return 0;
}

/*****************************************************************************
 函 数 名  : uart_fetch_sshell_buf
 功能描述  : 取sshell数据缓存
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
 函 数 名  : _print_thread
 功能描述  : 打印线程，优先级为LOW
 输入参数  : void *arg  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

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
 函 数 名  : uart_fml_init
 功能描述  : 串口功能模块初始化
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
void uart_fml_init(void)
{
	osThreadId_t thread_print_id = 0;
	
	print_rngbuf = rng_create(SIZE_8K);	// 8KB缓存
	sshell_rngbuf = rng_create(SIZE_256);
    receiver_rngbuf = rng_create(SIZE_1K);
    if(!receiver_rngbuf)
        ERR_PRINT(("receiver_rngbuf create failed!!!\r\n"));
	uart_register_rx_callbak(DEBUG_PRINT_COM, _uart_rcv_sshell_data);	//注册调试串口
	//uart_register_rx_callbak(DEBUG_PRINT_COM_BAK, _uart_rcv_sshell_data);	//注册调试串口
	thread_print_id = osThreadNew(_print_thread, NULL, &thread_print_attr);
	GLOBAL_HEX(thread_print_id);
}

/*eof*/
