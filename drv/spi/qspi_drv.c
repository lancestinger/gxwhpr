#include "spi/qspi_drv.h"
//#include "delay.h"

QSPI_HandleTypeDef qspi_handler;    //QSPI���
    
//QSPI��ʼ��
U8 qspi_drv_init(void)
{
    qspi_handler.Instance=QUADSPI;                          //QSPI
    qspi_handler.Init.ClockPrescaler=1;                     //QPSI��Ƶ�ȣ�W25Q256���Ƶ��Ϊ104M��
                                                            //���Դ˴�Ӧ��Ϊ2��QSPIƵ�ʾ�Ϊ200/(1+1)=100MHZ
    qspi_handler.Init.FifoThreshold=32;                      //FIFO��ֵΪ4���ֽ�
    qspi_handler.Init.SampleShifting=QSPI_SAMPLE_SHIFTING_HALFCYCLE;//������λ�������(DDRģʽ��,��������Ϊ0)
    qspi_handler.Init.FlashSize=POSITION_VAL(0X2000000)-1;  //SPI FLASH��С��W25Q256��СΪ32M�ֽ�
    qspi_handler.Init.ChipSelectHighTime=QSPI_CS_HIGH_TIME_1_CYCLE;//Ƭѡ�ߵ�ƽʱ��Ϊ5��ʱ��(10*5=55ns),���ֲ������tSHSL����
    qspi_handler.Init.ClockMode=QSPI_CLOCK_MODE_0;          //ģʽ0
    qspi_handler.Init.FlashID=QSPI_FLASH_ID_1;              //��һƬflash
    qspi_handler.Init.DualFlash=QSPI_DUALFLASH_DISABLE;     //��ֹ˫����ģʽ
    if(HAL_QSPI_Init(&qspi_handler)==HAL_OK) return 0;      //QSPI��ʼ���ɹ�
    else return 1;
}

//QSPI�ײ�����,�������ã�ʱ��ʹ��
//�˺����ᱻHAL_qspi_drv_init()����
//hqspi:QSPI���
void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
    GPIO_InitTypeDef gpio_initure;
    
    __HAL_RCC_QSPI_CLK_ENABLE();        //ʹ��QSPIʱ��
    __HAL_RCC_GPIOB_CLK_ENABLE();       //ʹ��GPIOBʱ��
    __HAL_RCC_GPIOF_CLK_ENABLE();       //ʹ��GPIOFʱ��
    
    //��ʼ��PB6 Ƭѡ�ź�
    gpio_initure.Pin=GPIO_PIN_6;
    gpio_initure.Mode=GPIO_MODE_AF_PP;          //����
    gpio_initure.Pull=GPIO_PULLUP;              
    gpio_initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  //����
    gpio_initure.Alternate=GPIO_AF10_QUADSPI;   //����ΪQSPI
    HAL_GPIO_Init(GPIOB,&gpio_initure);
    
    //PF8,9
    gpio_initure.Pin=GPIO_PIN_8|GPIO_PIN_9;
    gpio_initure.Pull=GPIO_NOPULL;              
    gpio_initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   //����
    HAL_GPIO_Init(GPIOF,&gpio_initure);
    
    //PB2
    gpio_initure.Pin=GPIO_PIN_2;
    gpio_initure.Alternate=GPIO_AF9_QUADSPI;   //����ΪQSPI
    HAL_GPIO_Init(GPIOB,&gpio_initure);
    
    //PF6,7
    gpio_initure.Pin=GPIO_PIN_6|GPIO_PIN_7;
    HAL_GPIO_Init(GPIOF,&gpio_initure);
}

//QSPI��������
//instruction:Ҫ���͵�ָ��
//address:���͵���Ŀ�ĵ�ַ
//dummyCycles:��ָ��������
//	instructionMode:ָ��ģʽ;QSPI_INSTRUCTION_NONE,QSPI_INSTRUCTION_1_LINE,QSPI_INSTRUCTION_2_LINE,QSPI_INSTRUCTION_4_LINE  
//	addressMode:��ַģʽ; QSPI_ADDRESS_NONE,QSPI_ADDRESS_1_LINE,QSPI_ADDRESS_2_LINE,QSPI_ADDRESS_4_LINE
//	addressSize:��ַ����;QSPI_ADDRESS_8_BITS,QSPI_ADDRESS_16_BITS,QSPI_ADDRESS_24_BITS,QSPI_ADDRESS_32_BITS
//	dataMode:����ģʽ; QSPI_DATA_NONE,QSPI_DATA_1_LINE,QSPI_DATA_2_LINE,QSPI_DATA_4_LINE

void qspi_send_cmd(U32 instruction,U32 address,U32 dummyCycles,U32 instructionMode,U32 addressMode,U32 addressSize,U32 dataMode)
{
    QSPI_CommandTypeDef Cmdhandler;
    
    Cmdhandler.Instruction=instruction;                 	//ָ��
    Cmdhandler.Address=address;                            	//��ַ
    Cmdhandler.DummyCycles=dummyCycles;                     //���ÿ�ָ��������
    Cmdhandler.InstructionMode=instructionMode;				//ָ��ģʽ
    Cmdhandler.AddressMode=addressMode;   					//��ַģʽ
    Cmdhandler.AddressSize=addressSize;   					//��ַ����
    Cmdhandler.DataMode=dataMode;             				//����ģʽ
    Cmdhandler.SIOOMode=QSPI_SIOO_INST_EVERY_CMD;       	//ÿ�ζ�����ָ��
    Cmdhandler.AlternateByteMode=QSPI_ALTERNATE_BYTES_NONE; //�޽����ֽ�
    Cmdhandler.DdrMode=QSPI_DDR_MODE_DISABLE;           	//�ر�DDRģʽ
    Cmdhandler.DdrHoldHalfCycle=QSPI_DDR_HHC_HALF_CLK_DELAY;

    HAL_QSPI_Command(&qspi_handler,&Cmdhandler,5000);
}

//QSPI����ָ�����ȵ�����
//buf:�������ݻ������׵�ַ
//datalen:Ҫ��������ݳ���
//����ֵ:0,����
//    ����,�������
U8 qspi_receive(U8* buf,U32 datalen)
{
    qspi_handler.Instance->DLR=datalen-1;                           //�������ݳ���
    if(HAL_QSPI_Receive(&qspi_handler,buf,5000)==HAL_OK) return 0;  //��������
    else return 1;
}

//QSPI����ָ�����ȵ�����
//buf:�������ݻ������׵�ַ
//datalen:Ҫ��������ݳ���
//����ֵ:0,����
//    ����,�������
U8 qspi_transmit(U8* buf,U32 datalen)
{
    qspi_handler.Instance->DLR=datalen-1;                            //�������ݳ���
    if(HAL_QSPI_Transmit(&qspi_handler,buf,5000)==HAL_OK) return 0;  //��������
    else return 1;
}
