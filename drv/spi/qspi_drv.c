#include "spi/qspi_drv.h"
//#include "delay.h"

QSPI_HandleTypeDef qspi_handler;    //QSPI句柄
    
//QSPI初始化
U8 qspi_drv_init(void)
{
    qspi_handler.Instance=QUADSPI;                          //QSPI
    qspi_handler.Init.ClockPrescaler=1;                     //QPSI分频比，W25Q256最大频率为104M，
                                                            //所以此处应该为2，QSPI频率就为200/(1+1)=100MHZ
    qspi_handler.Init.FifoThreshold=32;                      //FIFO阈值为4个字节
    qspi_handler.Init.SampleShifting=QSPI_SAMPLE_SHIFTING_HALFCYCLE;//采样移位半个周期(DDR模式下,必须设置为0)
    qspi_handler.Init.FlashSize=POSITION_VAL(0X2000000)-1;  //SPI FLASH大小，W25Q256大小为32M字节
    qspi_handler.Init.ChipSelectHighTime=QSPI_CS_HIGH_TIME_1_CYCLE;//片选高电平时间为5个时钟(10*5=55ns),即手册里面的tSHSL参数
    qspi_handler.Init.ClockMode=QSPI_CLOCK_MODE_0;          //模式0
    qspi_handler.Init.FlashID=QSPI_FLASH_ID_1;              //第一片flash
    qspi_handler.Init.DualFlash=QSPI_DUALFLASH_DISABLE;     //禁止双闪存模式
    if(HAL_QSPI_Init(&qspi_handler)==HAL_OK) return 0;      //QSPI初始化成功
    else return 1;
}

//QSPI底层驱动,引脚配置，时钟使能
//此函数会被HAL_qspi_drv_init()调用
//hqspi:QSPI句柄
void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
    GPIO_InitTypeDef gpio_initure;
    
    __HAL_RCC_QSPI_CLK_ENABLE();        //使能QSPI时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();       //使能GPIOB时钟
    __HAL_RCC_GPIOF_CLK_ENABLE();       //使能GPIOF时钟
    
    //初始化PB6 片选信号
    gpio_initure.Pin=GPIO_PIN_6;
    gpio_initure.Mode=GPIO_MODE_AF_PP;          //复用
    gpio_initure.Pull=GPIO_PULLUP;              
    gpio_initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  //高速
    gpio_initure.Alternate=GPIO_AF10_QUADSPI;   //复用为QSPI
    HAL_GPIO_Init(GPIOB,&gpio_initure);
    
    //PF8,9
    gpio_initure.Pin=GPIO_PIN_8|GPIO_PIN_9;
    gpio_initure.Pull=GPIO_NOPULL;              
    gpio_initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   //高速
    HAL_GPIO_Init(GPIOF,&gpio_initure);
    
    //PB2
    gpio_initure.Pin=GPIO_PIN_2;
    gpio_initure.Alternate=GPIO_AF9_QUADSPI;   //复用为QSPI
    HAL_GPIO_Init(GPIOB,&gpio_initure);
    
    //PF6,7
    gpio_initure.Pin=GPIO_PIN_6|GPIO_PIN_7;
    HAL_GPIO_Init(GPIOF,&gpio_initure);
}

//QSPI发送命令
//instruction:要发送的指令
//address:发送到的目的地址
//dummyCycles:空指令周期数
//	instructionMode:指令模式;QSPI_INSTRUCTION_NONE,QSPI_INSTRUCTION_1_LINE,QSPI_INSTRUCTION_2_LINE,QSPI_INSTRUCTION_4_LINE  
//	addressMode:地址模式; QSPI_ADDRESS_NONE,QSPI_ADDRESS_1_LINE,QSPI_ADDRESS_2_LINE,QSPI_ADDRESS_4_LINE
//	addressSize:地址长度;QSPI_ADDRESS_8_BITS,QSPI_ADDRESS_16_BITS,QSPI_ADDRESS_24_BITS,QSPI_ADDRESS_32_BITS
//	dataMode:数据模式; QSPI_DATA_NONE,QSPI_DATA_1_LINE,QSPI_DATA_2_LINE,QSPI_DATA_4_LINE

void qspi_send_cmd(U32 instruction,U32 address,U32 dummyCycles,U32 instructionMode,U32 addressMode,U32 addressSize,U32 dataMode)
{
    QSPI_CommandTypeDef Cmdhandler;
    
    Cmdhandler.Instruction=instruction;                 	//指令
    Cmdhandler.Address=address;                            	//地址
    Cmdhandler.DummyCycles=dummyCycles;                     //设置空指令周期数
    Cmdhandler.InstructionMode=instructionMode;				//指令模式
    Cmdhandler.AddressMode=addressMode;   					//地址模式
    Cmdhandler.AddressSize=addressSize;   					//地址长度
    Cmdhandler.DataMode=dataMode;             				//数据模式
    Cmdhandler.SIOOMode=QSPI_SIOO_INST_EVERY_CMD;       	//每次都发送指令
    Cmdhandler.AlternateByteMode=QSPI_ALTERNATE_BYTES_NONE; //无交替字节
    Cmdhandler.DdrMode=QSPI_DDR_MODE_DISABLE;           	//关闭DDR模式
    Cmdhandler.DdrHoldHalfCycle=QSPI_DDR_HHC_HALF_CLK_DELAY;

    HAL_QSPI_Command(&qspi_handler,&Cmdhandler,5000);
}

//QSPI接收指定长度的数据
//buf:接收数据缓冲区首地址
//datalen:要传输的数据长度
//返回值:0,正常
//    其他,错误代码
U8 qspi_receive(U8* buf,U32 datalen)
{
    qspi_handler.Instance->DLR=datalen-1;                           //配置数据长度
    if(HAL_QSPI_Receive(&qspi_handler,buf,5000)==HAL_OK) return 0;  //接收数据
    else return 1;
}

//QSPI发送指定长度的数据
//buf:发送数据缓冲区首地址
//datalen:要传输的数据长度
//返回值:0,正常
//    其他,错误代码
U8 qspi_transmit(U8* buf,U32 datalen)
{
    qspi_handler.Instance->DLR=datalen-1;                            //配置数据长度
    if(HAL_QSPI_Transmit(&qspi_handler,buf,5000)==HAL_OK) return 0;  //发送数据
    else return 1;
}
