#ifndef __QSPI_H
#define __QSPI_H
#include "pubdef.h"

extern QSPI_HandleTypeDef QSPI_Handler;    //QSPI���

u8 qspi_drv_init(void);												//��ʼ��QSPI
void qspi_send_cmd(u32 instruction,u32 address,u32 dummyCycles,u32 instructionMode,u32 addressMode,u32 addressSize,u32 dataMode);			//QSPI��������
u8 qspi_receive(u8* buf,u32 datalen);							//QSPI��������
u8 qspi_transmit(u8* buf,u32 datalen);							//QSPI��������
#endif
