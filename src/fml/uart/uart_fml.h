
#ifndef _UART_FML_H_
#define _UART_FML_H_

#include "uart/uart_drv.h"


#define DEBUG_PRINT_COM		UART_COM1	//调试打印串口
#define DEBUG_PRINT_COM_BAK	UART_COM6	//备用调试打印串口
#define GNSS_COM			UART_COM7	//GNSS通信串口
#define MXT_COM             UART_COM4   //MXT906B串口

typedef enum
{
	DBG_OUT_USART = 0,
	DBG_OUT_TELNET,
	DBG_OUT_MQTT,
}DBG_OUT_Enum;

extern void print_def(CHAR_T * fmt ,...);
extern void uart_set_dbg_outmode(U8 mode);
extern U8 uart_get_dbg_outmode(void);
extern CHAR_T* uart_fetch_sshell_buf(void);
extern void uart_fml_init(void);
#endif

/*EOF*/

