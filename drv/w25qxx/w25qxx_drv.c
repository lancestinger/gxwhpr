/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : w25qxx.c
  �� �� ��   : ����
  ��    ��   : wenzhiquan
  ��������   : 2018��9��12�� ������
  ����޸�   :
  ��������   : SPI FLASH�����ļ�,��������Դ������
  �����б�   :
              w25qxx_erase_chip
              w25qxx_erase_sector
              w25qxx_drv_init
              W25QXX_PowerDown
              w25qxx_read
              w25qxx_read_id
              w25qxx_read_sr
              w25qxx_wait_busy
              W25QXX_WAKEUP
              w25qxx_write
              w25qxx_write_disable
              w25qxx_write_enable
              w25qxx_write_nocheck
              w25qxx_write_page
              w25qxx_write_sr
  �޸���ʷ   :
  1.��    ��   : 2018��9��12�� ������
    ��    ��   : wenzhiquan
    �޸�����   : �����ļ�

******************************************************************************/

#include "w25qxx/w25qxx_drv.h"
#include "spi/qspi_drv.h"
#include "Driver_Flash.h"
#include "spi/spi_drv.h"


#define ARM_FLASH_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,2) /* driver version */

//#define DRIVER_USE_QSPI

#define W25QXX_SPI	SPI_2

#ifndef DRIVER_FLASH_NUM
#define DRIVER_FLASH_NUM        0       /* Default driver number */
#endif



static U16 w25qxx_type = 0;	//Ĭ����W25Q256
#ifdef DRIVER_USE_QSPI
static U8 w25qxx_qpi_mode=0;		//QSPIģʽ��־:0,SPIģʽ;1,QPIģʽ.
#endif

//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q256
//����Ϊ32M�ֽ�,����512��Block,8192��Sector 
													 
//��ʼ��SPI FLASH��IO��
U8 w25qxx_drv_init(void)
{ 
	U8 temp; 
	U8 ret = FALSE;

	#ifdef DRIVER_USE_QSPI
	qspi_drv_init();					//��ʼ��QSPI
	w25qxx_qspi_enable();			//ʹ��QSPIģʽ
	w25qxx_type=w25qxx_read_id();	//��ȡFLASH ID.
	//printf("ID:%x\r\n",w25qxx_type);
	if(w25qxx_type==W25Q256)        //SPI FLASHΪW25Q256
	{
	    temp=w25qxx_read_sr(3);      //��ȡ״̬�Ĵ���3���жϵ�ַģʽ
	    if((temp&0X01)==0)			//�������4�ֽڵ�ַģʽ,�����4�ֽڵ�ַģʽ
		{ 
			w25qxx_write_enable();	//дʹ��
			qspi_send_cmd(W25X_Enable4ByteAddr,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//QPI,ʹ��4�ֽڵ�ַָ��,��ַΪ0,������_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿�����,0���ֽ����� 
		}
		w25qxx_write_enable();		//дʹ��
		qspi_send_cmd(W25X_SetReadParam,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_4_LINES); 		//QPI,���ö�����ָ��,��ַΪ0,4�ߴ�����_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿�����,1���ֽ�����
		temp=3<<4;					//����P4&P5=11,8��dummy clocks,104M
		qspi_transmit(&temp,1);		//����1���ֽ�	   
		ret = TRUE;
	}
	#else
	w25qxx_type=w25qxx_read_id();	//��ȡFLASH ID.
	NOTE_PRINT(("ID:%x\r\n",w25qxx_type));
	if(w25qxx_type==W25Q256)        //SPI FLASHΪW25Q256
	{
		temp=w25qxx_read_sr(3);              //��ȡ״̬�Ĵ���3���жϵ�ַģʽ
		if((temp&0X01)==0)			        //�������4�ֽڵ�ַģʽ,�����4�ֽڵ�ַģʽ
		{
			SPI2_CS_LOW; 			        //ѡ��
			spi_read_write_byte(W25QXX_SPI, W25X_Enable4ByteAddr);//���ͽ���4�ֽڵ�ַģʽָ��   
			SPI2_CS_HIGH;        		        //ȡ��Ƭѡ   
		}
		GLOBAL_TRACE(("FLASH W25Q256��ʼ�����!!!!!!\r\n"));
		ret = TRUE;
	}
	#endif
	return ret;
}  

#ifdef DRIVER_USE_QSPI
//W25QXX����QSPIģʽ 
void w25qxx_qspi_enable(void)
{
	U8 stareg2;
	stareg2=w25qxx_read_sr(2);		//�ȶ���״̬�Ĵ���2��ԭʼֵ
	if((stareg2&0X02)==0)			//QEλδʹ��
	{
		w25qxx_write_enable();		//дʹ�� 
		stareg2|=1<<1;				//ʹ��QEλ		
		w25qxx_write_sr(2,stareg2);	//д״̬�Ĵ���2
	}
	qspi_send_cmd(W25X_EnterQPIMode,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//дcommandָ��,��ַΪ0,������_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿�����,0���ֽ�����
	w25qxx_qpi_mode=1;				//���QSPIģʽ
}

//W25QXX�˳�QSPIģʽ 
void w25qxx_qspi_disable(void)
{ 
	qspi_send_cmd(W25X_ExitQPIMode,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//дcommandָ��,��ַΪ0,������_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿�����,0���ֽ�����
	w25qxx_qpi_mode=0;				//���SPIģʽ
}
#endif
//��ȡW25QXX��״̬�Ĵ�����W25QXXһ����3��״̬�Ĵ���
//״̬�Ĵ���1��
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
//״̬�Ĵ���2��
//BIT7  6   5   4   3   2   1   0
//SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
//״̬�Ĵ���3��
//BIT7      6    5    4   3   2   1   0
//HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
//regno:״̬�Ĵ����ţ���:1~3
//����ֵ:״̬�Ĵ���ֵ
U8 w25qxx_read_sr(U8 regno)   
{  
	U8 byte=0,command=0; 
    switch(regno)
    {
        case 1:
            command=W25X_ReadStatusReg1;    //��״̬�Ĵ���1ָ��
            break;
        case 2:
            command=W25X_ReadStatusReg2;    //��״̬�Ĵ���2ָ��
            break;
        case 3:
            command=W25X_ReadStatusReg3;    //��״̬�Ĵ���3ָ��
            break;
        default:
            command=W25X_ReadStatusReg1;    
            break;
    }   
	#ifdef DRIVER_USE_QSPI
	if(w25qxx_qpi_mode)qspi_send_cmd(command,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_4_LINES);	//QPI,дcommandָ��,��ַΪ0,4�ߴ�����_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿�����,1���ֽ�����
	else qspi_send_cmd(command,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_1_LINE);				//SPI,дcommandָ��,��ַΪ0,���ߴ�����_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿�����,1���ֽ�����
	qspi_receive(&byte,1);	 
	#else
	SPI2_CS_LOW;                            			//ʹ������   
	spi_read_write_byte(W25QXX_SPI, command);			//����дȡ״̬�Ĵ�������    
	byte = spi_read_write_byte(W25QXX_SPI, 0xFF);		//��ȡһ���ֽ�  
	SPI2_CS_HIGH;                            			//ȡ��Ƭѡ    
	#endif
	return byte;   
} 
//дW25QXX״̬�Ĵ���
void w25qxx_write_sr(U8 regno,U8 sr)   
{   
    U8 command=0;
    switch(regno)
    {
        case 1:
            command=W25X_WriteStatusReg1;    //д״̬�Ĵ���1ָ��
            break;
        case 2:
            command=W25X_WriteStatusReg2;    //д״̬�Ĵ���2ָ��
            break;
        case 3:
            command=W25X_WriteStatusReg3;    //д״̬�Ĵ���3ָ��
            break;
        default:
            command=W25X_WriteStatusReg1;    
            break;
    }   
	#ifdef DRIVER_USE_QSPI
	if(w25qxx_qpi_mode)qspi_send_cmd(command,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_4_LINES);	//QPI,дcommandָ��,��ַΪ0,4�ߴ�����_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿�����,1���ֽ�����
	else qspi_send_cmd(command,0,0, QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_1_LINE);				//SPI,дcommandָ��,��ַΪ0,���ߴ�����_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿�����,1���ֽ�����
	qspi_transmit(&sr,1);	  
	#else
	SPI2_CS_LOW;                            		//ʹ������   
	spi_read_write_byte(W25QXX_SPI, command);		//����дȡ״̬�Ĵ�������    
	spi_read_write_byte(W25QXX_SPI, sr);         //д��һ���ֽ�  
	SPI2_CS_HIGH;                            //ȡ��Ƭѡ    
	#endif
}   
//W25QXXдʹ��	
//��WEL��λ   
void w25qxx_write_enable(void)   
{
	#ifdef DRIVER_USE_QSPI
	if(w25qxx_qpi_mode)qspi_send_cmd(W25X_WriteEnable,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);	//QPI,дʹ��ָ��,��ַΪ0,������_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿�����,0���ֽ�����
	else qspi_send_cmd(W25X_WriteEnable,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);				//SPI,дʹ��ָ��,��ַΪ0,������_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿�����,0���ֽ�����
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_WriteEnable);
	SPI2_CS_HIGH;
	#endif
} 

//W25QXXд��ֹ	
//��WEL����  
void w25qxx_write_disable(void)   
{  
	#ifdef DRIVER_USE_QSPI
	if(w25qxx_qpi_mode)qspi_send_cmd(W25X_WriteDisable,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//QPI,д��ָֹ��,��ַΪ0,������_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿�����,0���ֽ�����
	else qspi_send_cmd(W25X_WriteDisable,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);				//SPI,д��ָֹ��,��ַΪ0,������_8λ��ַ_�޵�ַ_���ߴ���ָ��,�޿�����,0���ֽ����� 
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_WriteDisable);
	SPI2_CS_HIGH;
	#endif
} 

//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
//0XEF18,��ʾоƬ�ͺ�ΪW25Q256
U16 w25qxx_read_id(void)
{
	U16 deviceid = 0;

	#ifdef DRIVER_USE_QSPI
	U8 temp[2];
	if(w25qxx_qpi_mode)qspi_send_cmd(W25X_ManufactDeviceID,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_24_BITS,QSPI_DATA_4_LINES);//QPI,��id,��ַΪ0,4�ߴ�������_24λ��ַ_4�ߴ����ַ_4�ߴ���ָ��,�޿�����,2���ֽ�����
	else qspi_send_cmd(W25X_ManufactDeviceID,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_1_LINE,QSPI_ADDRESS_24_BITS,QSPI_DATA_1_LINE);			//SPI,��id,��ַΪ0,���ߴ�������_24λ��ַ_���ߴ����ַ_���ߴ���ָ��,�޿�����,2���ֽ�����
	qspi_receive(temp,2);
	deviceid=(temp[0]<<8)|temp[1];
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, 0x90);
	spi_read_write_byte(W25QXX_SPI, 0x00);
	spi_read_write_byte(W25QXX_SPI, 0x00);
	spi_read_write_byte(W25QXX_SPI, 0x00);
	deviceid = (spi_read_write_byte(W25QXX_SPI, 0xFF) << 8);
	deviceid |= (spi_read_write_byte(W25QXX_SPI, 0xFF));
	
	/*SPI2_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
	SPI2_ReadWriteByte(0x00); 	    
	SPI2_ReadWriteByte(0x00); 	    
	SPI2_ReadWriteByte(0x00); 	 			   
	deviceid|=SPI2_ReadWriteByte(0xFF)<<8;  
	deviceid|=SPI2_ReadWriteByte(0xFF);	*/
	SPI2_CS_HIGH;
	#endif
	return deviceid;
}   		    
//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void w25qxx_read(U8* pBuffer,u32 ReadAddr,U16 NumByteToRead)   
{  	
	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_FastReadData,ReadAddr,8,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_32_BITS,QSPI_DATA_4_LINES);	//QPI,���ٶ�����,��ַΪReadAddr,4�ߴ�������_32λ��ַ_4�ߴ����ַ_4�ߴ���ָ��,8������,NumByteToRead������
	qspi_receive(pBuffer,NumByteToRead); 
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_ReadData);      //���Ͷ�ȡ����  
    if(w25qxx_type == W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        spi_read_write_byte(W25QXX_SPI, (u8)((ReadAddr)>>24));    
    }
    spi_read_write_byte(W25QXX_SPI, (u8)((ReadAddr)>>16));   //����24bit��ַ    
    spi_read_write_byte(W25QXX_SPI, (u8)((ReadAddr)>>8));   
    spi_read_write_byte(W25QXX_SPI, (u8)ReadAddr);   
    spi_read_buffer(W25QXX_SPI, pBuffer, NumByteToRead);	//д����
	SPI2_CS_HIGH;
	#endif
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void w25qxx_write_page(U8* pBuffer,u32 WriteAddr,U16 NumByteToWrite)
{  
 	w25qxx_write_enable();					//дʹ��
 	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_PageProgram,WriteAddr,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_32_BITS,QSPI_DATA_4_LINES);	//QPI,ҳдָ��,��ַΪWriteAddr,4�ߴ�������_32λ��ַ_4�ߴ����ַ_4�ߴ���ָ��,�޿�����,NumByteToWrite������
	qspi_transmit(pBuffer,NumByteToWrite);	   
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_PageProgram);   //����дҳ����   
    if(w25qxx_type == W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        spi_read_write_byte(W25QXX_SPI, (u8)((WriteAddr)>>24)); 
    }
    spi_read_write_byte(W25QXX_SPI, (u8)((WriteAddr)>>16)); //����24bit��ַ    
    spi_read_write_byte(W25QXX_SPI, (u8)((WriteAddr)>>8));   
    spi_read_write_byte(W25QXX_SPI, (u8)WriteAddr);   
	spi_write_buffer(W25QXX_SPI, pBuffer, NumByteToWrite);
	SPI2_CS_HIGH;
	#endif
	w25qxx_wait_busy();					   //�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void w25qxx_write_nocheck(U8* pBuffer,u32 WriteAddr,U16 NumByteToWrite)   
{ 			 		 
	U16 pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		w25qxx_write_page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	}   
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
U8 w25qxx_buffer[4096];		 
void w25qxx_write(U8* pBuffer,u32 WriteAddr,U16 NumByteToWrite)   
{ 
	u32 secpos;
	U16 secoff;
	U16 secremain;	   
 	U16 i;    
	U8 * w25qxx_pbuf;	  
   	w25qxx_pbuf=w25qxx_buffer;	     
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		w25qxx_read(w25qxx_pbuf,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(w25qxx_pbuf[secoff+i]!=0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			w25qxx_erase_sector(secpos);//�����������
			for(i=0;i<secremain;i++)	   //����
			{
				w25qxx_pbuf[i+secoff]=pBuffer[i];	  
			}
			w25qxx_write_nocheck(w25qxx_pbuf,secpos*4096,4096);//д����������  

		}else w25qxx_write_nocheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

		   	pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��	   
		   	NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
		}	 
	};	 
}
//��������оƬ		  
//�ȴ�ʱ�䳬��...
void w25qxx_erase_chip(void)   
{                                   
    w25qxx_write_enable();					//SET WEL 
    w25qxx_wait_busy();   
	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_ChipErase,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//QPI,дȫƬ����ָ��,��ַΪ0,������_8λ��ַ_�޵�ַ_4�ߴ���ָ��,�޿�����,0���ֽ�����
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_ChipErase);        //����Ƭ��������  
	SPI2_CS_HIGH;
	#endif
	while((w25qxx_read_sr(1)&0x01)==0x01)	//�ȴ��������
	{
		delay_ms(10);   // �ȴ�BUSYλ���
	}
	//w25qxx_wait_busy();   				   //�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ������������ʱ��:150ms
void w25qxx_erase_sector(u32 Dst_Addr)   
{  
 	Dst_Addr*=4096;
    w25qxx_write_enable();                  //SET WEL 	 
    w25qxx_wait_busy();  
	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_SectorErase,Dst_Addr,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_32_BITS,QSPI_DATA_NONE);//QPI,д��������ָ��,��ַΪ0,������_32λ��ַ_4�ߴ����ַ_4�ߴ���ָ��,�޿�����,0���ֽ�����
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_SectorErase);        //����������������  
	if(w25qxx_type == W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        spi_read_write_byte(W25QXX_SPI, (u8)((Dst_Addr)>>24)); 
    }
    spi_read_write_byte(W25QXX_SPI, (u8)((Dst_Addr)>>16));  //����24bit��ַ    
    spi_read_write_byte(W25QXX_SPI, (u8)((Dst_Addr)>>8));   
    spi_read_write_byte(W25QXX_SPI, (u8)Dst_Addr);  
	SPI2_CS_HIGH;
	#endif
	w25qxx_wait_busy();   				    //�ȴ��������
}  
//�ȴ�����
void w25qxx_wait_busy(void)   
{   
	while((w25qxx_read_sr(1)&0x01)==0x01)
	{
		//delay_ms(1);   // �ȴ�BUSYλ���
	}
}   


/* Send command with optional data and wait until busy */
/*static bool SendCommand (uint8_t cmd, uint32_t addr, const uint8_t *data, uint32_t size) 
{
 
  return FALSE;
}
*/
/* Sector Information */
#ifdef FLASH_SECTORS
static ARM_FLASH_SECTOR flash_sector_info[FLASH_BLOCK_COUNT]=
{
	FLASH_SECTORS
	
};
#else
#define flash_sector_info NULL
#endif

/*static const ARM_FLASH_SECTOR UPDATE_SECTOR_INFO[UPDATE_SEC_NUM]=
{
	UPDATE_SECTORS
};
*/
/* Flash Information */
static ARM_FLASH_INFO flash_info = {
  flash_sector_info,
  FLASH_BLOCK_COUNT,
  FLASH_BLOCK_SIZE,
  FLASH_PAGE_SIZE,
  FLASH_PROGRAM_UNIT,
  FLASH_ERASED_VALUE
};



/* Flash Status */
static ARM_FLASH_STATUS flash_status;


/* Driver Version */
static const ARM_DRIVER_VERSION driver_version = {
  ARM_FLASH_API_VERSION,
  ARM_FLASH_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_FLASH_CAPABILITIES driver_capabilities = {
  0,    /* event_ready */
  0,    /* data_width = 0:8-bit, 1:16-bit, 2:32-bit */
  1     /* erase_chip */
};


/**
  \fn          ARM_DRIVER_VERSION ARM_Flash_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION _get_version (void) {
  return driver_version;
}

/**
  \fn          ARM_FLASH_CAPABILITIES ARM_Flash_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_FLASH_CAPABILITIES
*/
static ARM_FLASH_CAPABILITIES _get_capabilities (void) {
  return driver_capabilities;
}

/**
  \fn          int32_t ARM_Flash_Initialize (ARM_Flash_SignalEvent_t cb_event)
  \brief       _initialize the Flash Interface.
  \param[in]   cb_event  Pointer to \ref ARM_Flash_SignalEvent
  \return      \ref execution_status
*/
static int32_t _initialize (ARM_Flash_SignalEvent_t cb_event) 
{
	if(w25qxx_drv_init() == TRUE)
	{
		flash_status.busy = 0;
		flash_status.error = 0;
  		return ARM_DRIVER_OK;
	}
	else
	{
		ERR_PRINT(("w25qxx_drv_init fault!!!!\r\n"));
		return ARM_DRIVER_ERROR;	
	}
}

/**
  \fn          int32_t ARM_Flash_Uninitialize (void)
  \brief       De-initialize the Flash Interface.
  \return      \ref execution_status
*/
static int32_t _uninitialize (void) {

  //result = ptrSPI->_uninitialize();
  //if (result != ARM_DRIVER_OK) return ARM_DRIVER_ERROR;

  return ARM_DRIVER_OK;
} 

/**
  \fn          int32_t ARM_Flash_PowerControl (ARM_POWER_STATE state)
  \brief       Control the Flash interface power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t _power_control (ARM_POWER_STATE state) {
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_ReadData (uint32_t addr, void *data, uint32_t cnt)
  \brief       Read data from Flash.
  \param[in]   addr  Data address.
  \param[out]  data  Pointer to a buffer storing the data read from Flash.
  \param[in]   cnt   Number of data items to read.
  \return      number of data items read or \ref execution_status
*/
static int32_t _read_data (uint32_t addr, void *data, uint32_t cnt) 
{	
	if ((data == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
	flash_status.busy = 1;
	//flash_id = w25qxx_read_id();
	w25qxx_read((U8*)data, addr, cnt);
	/*{
		U16 i;  
		U8* pBuffer = (U8*)data;
		
		W25QXX_CS_L;                            //ʹ������   
	    SPI1_ReadWriteByte(W25X_ReadData);      //���Ͷ�ȡ����  
	    if(w25qxx_type==W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
	    {
	        SPI1_ReadWriteByte((U8)((addr)>>24));    
	    }
	    SPI1_ReadWriteByte((U8)((addr)>>16));   //����24bit��ַ    
	    SPI1_ReadWriteByte((U8)((addr)>>8));   
	    SPI1_ReadWriteByte((U8)addr);   
	    for(i=0; i<cnt; i++)
		{ 
	        *((uint8_t*)data+i)=SPI1_ReadWriteByte(0XFF);    //ѭ������  
	    }
		W25QXX_CS_H; 
	}*/
	flash_status.busy = 0;
	return ARM_DRIVER_OK;
	//return (int32_t)cnt;
}

static int32_t _program_data (uint32_t addr, const void *buf, uint32_t sz) 
{
	flash_status.busy = 1;
	#if 1
	w25qxx_write_nocheck((U8*)buf, addr, sz);
	#else
	uint32_t cnt, i = 0;
  	uint8_t  sr;
	while(sz)
	{
		cnt = FLASH_PAGE_SIZE - (addr & (FLASH_PAGE_SIZE - 1));
		if (cnt > sz) cnt = sz;
		//MX25_WaitFlashReady(500000);
		//while(MX25_IsFlashBusy());
		w25qxx_write_enable();                  //SET WEL 
		W25QXX_CS_L;                            //ʹ������   
	    SPI1_ReadWriteByte(W25X_PageProgram);   //����дҳ����   
	    if(w25qxx_type==W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
	    {
	        SPI1_ReadWriteByte((U8)((addr)>>24)); 
	    }
	    SPI1_ReadWriteByte((U8)((addr)>>16)); //����24bit��ַ    
	    SPI1_ReadWriteByte((U8)((addr)>>8));   
	    SPI1_ReadWriteByte((U8)addr);   
	    for(i = 0;i < cnt;i++)
	    {
	        SPI1_ReadWriteByte(*((uint8_t*)buf+i));
	    }
		W25QXX_CS_H;                            //ȡ��Ƭѡ 
		w25qxx_wait_busy();					   //�ȴ�д�����
		addr += cnt;
	    ((uint8_t*)buf) += cnt;
	    sz  -= cnt;
	}
	#endif
	flash_status.busy = 0;
	return ARM_DRIVER_OK;
	//return ((int32_t)cnt - 1);
}

/**
  \fn          int32_t ARM_Flash_EraseSector (uint32_t addr)
  \brief       Erase Flash Sector.
  \param[in]   addr  Sector address
  \return      \ref execution_status
*/
static int32_t _erase_block (uint32_t addr) 
{
	flash_status.busy = 1;
	
	//addr*=FLASH_BLOCK_SIZE;
    w25qxx_write_enable();                  //SET WEL 
    while((w25qxx_read_sr(1)&0x01)==0x01)	//�ȴ��������
	{
		delay_ms(5);   // �ȴ�BUSYλ���
	}
	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_BlockErase,addr,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_32_BITS,QSPI_DATA_NONE);//QPI,д��������ָ��,��ַΪ0,������_32λ��ַ_4�ߴ����ַ_4�ߴ���ָ��,�޿�����,0���ֽ�����
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_BlockErase);   //������������ָ�� 
    if(w25qxx_type == W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        spi_read_write_byte(W25QXX_SPI, (u8)((addr)>>24)); 
    }
    spi_read_write_byte(W25QXX_SPI, (u8)((addr)>>16));  //����24bit��ַ    
    spi_read_write_byte(W25QXX_SPI, (u8)((addr)>>8));   
    spi_read_write_byte(W25QXX_SPI, (u8)addr);  
	SPI2_CS_HIGH;
	#endif
	
    while((w25qxx_read_sr(1)&0x01)==0x01)	//�ȴ��������
	{
		delay_ms(5);   // �ȴ�BUSYλ���
	}
	flash_status.busy = 0;
	return ARM_DRIVER_OK;
}



/**
  \fn          int32_t ARM_Flash_EraseChip (void)
  \brief       Erase complete Flash.
               Optional function for faster full chip erase.
  \return      \ref execution_status
*/
static int32_t _erase_chip (void) 
{
	flash_status.busy = 1;
	w25qxx_erase_chip();
	flash_status.busy = 0;
	return ARM_DRIVER_OK;
}


/**
  \fn          ARM_FLASH_STATUS ARM_Flash_GetStatus (void)
  \brief       Get Flash status.
  \return      Flash status \ref ARM_FLASH_STATUS
*/
static ARM_FLASH_STATUS _get_status (void) {
  return flash_status;
}

/**
  \fn          ARM_FLASH_INFO * ARM_Flash_GetInfo (void)
  \brief       Get Flash information.
  \return      Pointer to Flash information \ref ARM_FLASH_INFO
*/
static ARM_FLASH_INFO * _get_info (void) 
{

	/*for(i = 0; i < FLASH_BLOCK_COUNT; i++)
	{
		flash_sector_info[i].start = i*MX25_SECTOR_SIZE;
		flash_sector_info[i].end = i*MX25_SECTOR_SIZE + MX25_SECTOR_SIZE-1;
		//memcpy(&flash_sector_info[i], &(ARM_FLASH_SECTOR_INFO(i*0x1000, 0x1000)), sizeof(flash_sector_info[i]));
	}*/
  	return &flash_info;
}


/* Flash Driver Control Block */
ARM_DRIVER_FLASH ARM_Driver_Flash_(DRIVER_FLASH_NUM) = {
  _get_version,
  _get_capabilities,
  _initialize,
  _uninitialize,
  _power_control,
  _read_data,
  _program_data,
  _erase_block,
  _erase_chip,
  _get_status,
  _get_info
};


void w25qxx_erase_fs_sector(void)
{
	int i = 0;

	NOTE_PRINT(("��ʼ�����ⲿFLASH!!!!!!!\r\n"));
	for(i = 0; i < FLASH_BLOCK_COUNT; i++)
	{
		_erase_block(flash_sector_info[i].start);
	}
	NOTE_PRINT(("��ɲ����ⲿFLASH!!!!!!!\r\n"));
	//_erase_block();
}


