/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : w25qxx.c
  版 本 号   : 初稿
  作    者   : wenzhiquan
  生成日期   : 2018年9月12日 星期三
  最近修改   :
  功能描述   : SPI FLASH程序文件,该驱动来源于网络
  函数列表   :
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
  修改历史   :
  1.日    期   : 2018年9月12日 星期三
    作    者   : wenzhiquan
    修改内容   : 创建文件

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



static U16 w25qxx_type = 0;	//默认是W25Q256
#ifdef DRIVER_USE_QSPI
static U8 w25qxx_qpi_mode=0;		//QSPI模式标志:0,SPI模式;1,QPI模式.
#endif

//4Kbytes为一个Sector
//16个扇区为1个Block
//W25Q256
//容量为32M字节,共有512个Block,8192个Sector 
													 
//初始化SPI FLASH的IO口
U8 w25qxx_drv_init(void)
{ 
	U8 temp; 
	U8 ret = FALSE;

	#ifdef DRIVER_USE_QSPI
	qspi_drv_init();					//初始化QSPI
	w25qxx_qspi_enable();			//使能QSPI模式
	w25qxx_type=w25qxx_read_id();	//读取FLASH ID.
	//printf("ID:%x\r\n",w25qxx_type);
	if(w25qxx_type==W25Q256)        //SPI FLASH为W25Q256
	{
	    temp=w25qxx_read_sr(3);      //读取状态寄存器3，判断地址模式
	    if((temp&0X01)==0)			//如果不是4字节地址模式,则进入4字节地址模式
		{ 
			w25qxx_write_enable();	//写使能
			qspi_send_cmd(W25X_Enable4ByteAddr,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//QPI,使能4字节地址指令,地址为0,无数据_8位地址_无地址_4线传输指令,无空周期,0个字节数据 
		}
		w25qxx_write_enable();		//写使能
		qspi_send_cmd(W25X_SetReadParam,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_4_LINES); 		//QPI,设置读参数指令,地址为0,4线传数据_8位地址_无地址_4线传输指令,无空周期,1个字节数据
		temp=3<<4;					//设置P4&P5=11,8个dummy clocks,104M
		qspi_transmit(&temp,1);		//发送1个字节	   
		ret = TRUE;
	}
	#else
	w25qxx_type=w25qxx_read_id();	//读取FLASH ID.
	NOTE_PRINT(("ID:%x\r\n",w25qxx_type));
	if(w25qxx_type==W25Q256)        //SPI FLASH为W25Q256
	{
		temp=w25qxx_read_sr(3);              //读取状态寄存器3，判断地址模式
		if((temp&0X01)==0)			        //如果不是4字节地址模式,则进入4字节地址模式
		{
			SPI2_CS_LOW; 			        //选中
			spi_read_write_byte(W25QXX_SPI, W25X_Enable4ByteAddr);//发送进入4字节地址模式指令   
			SPI2_CS_HIGH;        		        //取消片选   
		}
		GLOBAL_TRACE(("FLASH W25Q256初始化完成!!!!!!\r\n"));
		ret = TRUE;
	}
	#endif
	return ret;
}  

#ifdef DRIVER_USE_QSPI
//W25QXX进入QSPI模式 
void w25qxx_qspi_enable(void)
{
	U8 stareg2;
	stareg2=w25qxx_read_sr(2);		//先读出状态寄存器2的原始值
	if((stareg2&0X02)==0)			//QE位未使能
	{
		w25qxx_write_enable();		//写使能 
		stareg2|=1<<1;				//使能QE位		
		w25qxx_write_sr(2,stareg2);	//写状态寄存器2
	}
	qspi_send_cmd(W25X_EnterQPIMode,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//写command指令,地址为0,无数据_8位地址_无地址_单线传输指令,无空周期,0个字节数据
	w25qxx_qpi_mode=1;				//标记QSPI模式
}

//W25QXX退出QSPI模式 
void w25qxx_qspi_disable(void)
{ 
	qspi_send_cmd(W25X_ExitQPIMode,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//写command指令,地址为0,无数据_8位地址_无地址_4线传输指令,无空周期,0个字节数据
	w25qxx_qpi_mode=0;				//标记SPI模式
}
#endif
//读取W25QXX的状态寄存器，W25QXX一共有3个状态寄存器
//状态寄存器1：
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
//状态寄存器2：
//BIT7  6   5   4   3   2   1   0
//SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
//状态寄存器3：
//BIT7      6    5    4   3   2   1   0
//HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
//regno:状态寄存器号，范:1~3
//返回值:状态寄存器值
U8 w25qxx_read_sr(U8 regno)   
{  
	U8 byte=0,command=0; 
    switch(regno)
    {
        case 1:
            command=W25X_ReadStatusReg1;    //读状态寄存器1指令
            break;
        case 2:
            command=W25X_ReadStatusReg2;    //读状态寄存器2指令
            break;
        case 3:
            command=W25X_ReadStatusReg3;    //读状态寄存器3指令
            break;
        default:
            command=W25X_ReadStatusReg1;    
            break;
    }   
	#ifdef DRIVER_USE_QSPI
	if(w25qxx_qpi_mode)qspi_send_cmd(command,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_4_LINES);	//QPI,写command指令,地址为0,4线传数据_8位地址_无地址_4线传输指令,无空周期,1个字节数据
	else qspi_send_cmd(command,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_1_LINE);				//SPI,写command指令,地址为0,单线传数据_8位地址_无地址_单线传输指令,无空周期,1个字节数据
	qspi_receive(&byte,1);	 
	#else
	SPI2_CS_LOW;                            			//使能器件   
	spi_read_write_byte(W25QXX_SPI, command);			//发送写取状态寄存器命令    
	byte = spi_read_write_byte(W25QXX_SPI, 0xFF);		//读取一个字节  
	SPI2_CS_HIGH;                            			//取消片选    
	#endif
	return byte;   
} 
//写W25QXX状态寄存器
void w25qxx_write_sr(U8 regno,U8 sr)   
{   
    U8 command=0;
    switch(regno)
    {
        case 1:
            command=W25X_WriteStatusReg1;    //写状态寄存器1指令
            break;
        case 2:
            command=W25X_WriteStatusReg2;    //写状态寄存器2指令
            break;
        case 3:
            command=W25X_WriteStatusReg3;    //写状态寄存器3指令
            break;
        default:
            command=W25X_WriteStatusReg1;    
            break;
    }   
	#ifdef DRIVER_USE_QSPI
	if(w25qxx_qpi_mode)qspi_send_cmd(command,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_4_LINES);	//QPI,写command指令,地址为0,4线传数据_8位地址_无地址_4线传输指令,无空周期,1个字节数据
	else qspi_send_cmd(command,0,0, QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_1_LINE);				//SPI,写command指令,地址为0,单线传数据_8位地址_无地址_单线传输指令,无空周期,1个字节数据
	qspi_transmit(&sr,1);	  
	#else
	SPI2_CS_LOW;                            		//使能器件   
	spi_read_write_byte(W25QXX_SPI, command);		//发送写取状态寄存器命令    
	spi_read_write_byte(W25QXX_SPI, sr);         //写入一个字节  
	SPI2_CS_HIGH;                            //取消片选    
	#endif
}   
//W25QXX写使能	
//将WEL置位   
void w25qxx_write_enable(void)   
{
	#ifdef DRIVER_USE_QSPI
	if(w25qxx_qpi_mode)qspi_send_cmd(W25X_WriteEnable,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);	//QPI,写使能指令,地址为0,无数据_8位地址_无地址_4线传输指令,无空周期,0个字节数据
	else qspi_send_cmd(W25X_WriteEnable,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);				//SPI,写使能指令,地址为0,无数据_8位地址_无地址_单线传输指令,无空周期,0个字节数据
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_WriteEnable);
	SPI2_CS_HIGH;
	#endif
} 

//W25QXX写禁止	
//将WEL清零  
void w25qxx_write_disable(void)   
{  
	#ifdef DRIVER_USE_QSPI
	if(w25qxx_qpi_mode)qspi_send_cmd(W25X_WriteDisable,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//QPI,写禁止指令,地址为0,无数据_8位地址_无地址_4线传输指令,无空周期,0个字节数据
	else qspi_send_cmd(W25X_WriteDisable,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);				//SPI,写禁止指令,地址为0,无数据_8位地址_无地址_单线传输指令,无空周期,0个字节数据 
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_WriteDisable);
	SPI2_CS_HIGH;
	#endif
} 

//读取芯片ID
//返回值如下:				   
//0XEF13,表示芯片型号为W25Q80  
//0XEF14,表示芯片型号为W25Q16    
//0XEF15,表示芯片型号为W25Q32  
//0XEF16,表示芯片型号为W25Q64 
//0XEF17,表示芯片型号为W25Q128 	  
//0XEF18,表示芯片型号为W25Q256
U16 w25qxx_read_id(void)
{
	U16 deviceid = 0;

	#ifdef DRIVER_USE_QSPI
	U8 temp[2];
	if(w25qxx_qpi_mode)qspi_send_cmd(W25X_ManufactDeviceID,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_24_BITS,QSPI_DATA_4_LINES);//QPI,读id,地址为0,4线传输数据_24位地址_4线传输地址_4线传输指令,无空周期,2个字节数据
	else qspi_send_cmd(W25X_ManufactDeviceID,0,0,QSPI_INSTRUCTION_1_LINE,QSPI_ADDRESS_1_LINE,QSPI_ADDRESS_24_BITS,QSPI_DATA_1_LINE);			//SPI,读id,地址为0,单线传输数据_24位地址_单线传输地址_单线传输指令,无空周期,2个字节数据
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
	
	/*SPI2_ReadWriteByte(0x90);//发送读取ID命令	    
	SPI2_ReadWriteByte(0x00); 	    
	SPI2_ReadWriteByte(0x00); 	    
	SPI2_ReadWriteByte(0x00); 	 			   
	deviceid|=SPI2_ReadWriteByte(0xFF)<<8;  
	deviceid|=SPI2_ReadWriteByte(0xFF);	*/
	SPI2_CS_HIGH;
	#endif
	return deviceid;
}   		    
//读取SPI FLASH  
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void w25qxx_read(U8* pBuffer,u32 ReadAddr,U16 NumByteToRead)   
{  	
	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_FastReadData,ReadAddr,8,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_32_BITS,QSPI_DATA_4_LINES);	//QPI,快速读数据,地址为ReadAddr,4线传输数据_32位地址_4线传输地址_4线传输指令,8空周期,NumByteToRead个数据
	qspi_receive(pBuffer,NumByteToRead); 
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_ReadData);      //发送读取命令  
    if(w25qxx_type == W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        spi_read_write_byte(W25QXX_SPI, (u8)((ReadAddr)>>24));    
    }
    spi_read_write_byte(W25QXX_SPI, (u8)((ReadAddr)>>16));   //发送24bit地址    
    spi_read_write_byte(W25QXX_SPI, (u8)((ReadAddr)>>8));   
    spi_read_write_byte(W25QXX_SPI, (u8)ReadAddr);   
    spi_read_buffer(W25QXX_SPI, pBuffer, NumByteToRead);	//写缓存
	SPI2_CS_HIGH;
	#endif
}  
//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!	 
void w25qxx_write_page(U8* pBuffer,u32 WriteAddr,U16 NumByteToWrite)
{  
 	w25qxx_write_enable();					//写使能
 	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_PageProgram,WriteAddr,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_32_BITS,QSPI_DATA_4_LINES);	//QPI,页写指令,地址为WriteAddr,4线传输数据_32位地址_4线传输地址_4线传输指令,无空周期,NumByteToWrite个数据
	qspi_transmit(pBuffer,NumByteToWrite);	   
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_PageProgram);   //发送写页命令   
    if(w25qxx_type == W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        spi_read_write_byte(W25QXX_SPI, (u8)((WriteAddr)>>24)); 
    }
    spi_read_write_byte(W25QXX_SPI, (u8)((WriteAddr)>>16)); //发送24bit地址    
    spi_read_write_byte(W25QXX_SPI, (u8)((WriteAddr)>>8));   
    spi_read_write_byte(W25QXX_SPI, (u8)WriteAddr);   
	spi_write_buffer(W25QXX_SPI, pBuffer, NumByteToWrite);
	SPI2_CS_HIGH;
	#endif
	w25qxx_wait_busy();					   //等待写入结束
} 
//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void w25qxx_write_nocheck(U8* pBuffer,u32 WriteAddr,U16 NumByteToWrite)   
{ 			 		 
	U16 pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		w25qxx_write_page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	}   
} 
//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)						
//NumByteToWrite:要写入的字节数(最大65535)   
U8 w25qxx_buffer[4096];		 
void w25qxx_write(U8* pBuffer,u32 WriteAddr,U16 NumByteToWrite)   
{ 
	u32 secpos;
	U16 secoff;
	U16 secremain;	   
 	U16 i;    
	U8 * w25qxx_pbuf;	  
   	w25qxx_pbuf=w25qxx_buffer;	     
 	secpos=WriteAddr/4096;//扇区地址  
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1) 
	{	
		w25qxx_read(w25qxx_pbuf,secpos*4096,4096);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(w25qxx_pbuf[secoff+i]!=0XFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			w25qxx_erase_sector(secpos);//擦除这个扇区
			for(i=0;i<secremain;i++)	   //复制
			{
				w25qxx_pbuf[i+secoff]=pBuffer[i];	  
			}
			w25qxx_write_nocheck(w25qxx_pbuf,secpos*4096,4096);//写入整个扇区  

		}else w25qxx_write_nocheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumByteToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

		   	pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移	   
		   	NumByteToWrite-=secremain;				//字节数递减
			if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=NumByteToWrite;			//下一个扇区可以写完了
		}	 
	};	 
}
//擦除整个芯片		  
//等待时间超长...
void w25qxx_erase_chip(void)   
{                                   
    w25qxx_write_enable();					//SET WEL 
    w25qxx_wait_busy();   
	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_ChipErase,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//QPI,写全片擦除指令,地址为0,无数据_8位地址_无地址_4线传输指令,无空周期,0个字节数据
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_ChipErase);        //发送片擦除命令  
	SPI2_CS_HIGH;
	#endif
	while((w25qxx_read_sr(1)&0x01)==0x01)	//等待擦除完成
	{
		delay_ms(10);   // 等待BUSY位清空
	}
	//w25qxx_wait_busy();   				   //等待芯片擦除结束
}   
//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个扇区的最少时间:150ms
void w25qxx_erase_sector(u32 Dst_Addr)   
{  
 	Dst_Addr*=4096;
    w25qxx_write_enable();                  //SET WEL 	 
    w25qxx_wait_busy();  
	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_SectorErase,Dst_Addr,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_32_BITS,QSPI_DATA_NONE);//QPI,写扇区擦除指令,地址为0,无数据_32位地址_4线传输地址_4线传输指令,无空周期,0个字节数据
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_SectorErase);        //发送扇区擦除命令  
	if(w25qxx_type == W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        spi_read_write_byte(W25QXX_SPI, (u8)((Dst_Addr)>>24)); 
    }
    spi_read_write_byte(W25QXX_SPI, (u8)((Dst_Addr)>>16));  //发送24bit地址    
    spi_read_write_byte(W25QXX_SPI, (u8)((Dst_Addr)>>8));   
    spi_read_write_byte(W25QXX_SPI, (u8)Dst_Addr);  
	SPI2_CS_HIGH;
	#endif
	w25qxx_wait_busy();   				    //等待擦除完成
}  
//等待空闲
void w25qxx_wait_busy(void)   
{   
	while((w25qxx_read_sr(1)&0x01)==0x01)
	{
		//delay_ms(1);   // 等待BUSY位清空
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
		
		W25QXX_CS_L;                            //使能器件   
	    SPI1_ReadWriteByte(W25X_ReadData);      //发送读取命令  
	    if(w25qxx_type==W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
	    {
	        SPI1_ReadWriteByte((U8)((addr)>>24));    
	    }
	    SPI1_ReadWriteByte((U8)((addr)>>16));   //发送24bit地址    
	    SPI1_ReadWriteByte((U8)((addr)>>8));   
	    SPI1_ReadWriteByte((U8)addr);   
	    for(i=0; i<cnt; i++)
		{ 
	        *((uint8_t*)data+i)=SPI1_ReadWriteByte(0XFF);    //循环读数  
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
		W25QXX_CS_L;                            //使能器件   
	    SPI1_ReadWriteByte(W25X_PageProgram);   //发送写页命令   
	    if(w25qxx_type==W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
	    {
	        SPI1_ReadWriteByte((U8)((addr)>>24)); 
	    }
	    SPI1_ReadWriteByte((U8)((addr)>>16)); //发送24bit地址    
	    SPI1_ReadWriteByte((U8)((addr)>>8));   
	    SPI1_ReadWriteByte((U8)addr);   
	    for(i = 0;i < cnt;i++)
	    {
	        SPI1_ReadWriteByte(*((uint8_t*)buf+i));
	    }
		W25QXX_CS_H;                            //取消片选 
		w25qxx_wait_busy();					   //等待写入结束
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
    while((w25qxx_read_sr(1)&0x01)==0x01)	//等待擦除完成
	{
		delay_ms(5);   // 等待BUSY位清空
	}
	#ifdef DRIVER_USE_QSPI
	qspi_send_cmd(W25X_BlockErase,addr,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_4_LINES,QSPI_ADDRESS_32_BITS,QSPI_DATA_NONE);//QPI,写扇区擦除指令,地址为0,无数据_32位地址_4线传输地址_4线传输指令,无空周期,0个字节数据
	#else
	SPI2_CS_LOW;
	spi_read_write_byte(W25QXX_SPI, W25X_BlockErase);   //发送扇区擦除指令 
    if(w25qxx_type == W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        spi_read_write_byte(W25QXX_SPI, (u8)((addr)>>24)); 
    }
    spi_read_write_byte(W25QXX_SPI, (u8)((addr)>>16));  //发送24bit地址    
    spi_read_write_byte(W25QXX_SPI, (u8)((addr)>>8));   
    spi_read_write_byte(W25QXX_SPI, (u8)addr);  
	SPI2_CS_HIGH;
	#endif
	
    while((w25qxx_read_sr(1)&0x01)==0x01)	//等待擦除完成
	{
		delay_ms(5);   // 等待BUSY位清空
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

	NOTE_PRINT(("开始擦除外部FLASH!!!!!!!\r\n"));
	for(i = 0; i < FLASH_BLOCK_COUNT; i++)
	{
		_erase_block(flash_sector_info[i].start);
	}
	NOTE_PRINT(("完成擦除外部FLASH!!!!!!!\r\n"));
	//_erase_block();
}


