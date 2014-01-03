/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery_l3g4200d.h"


/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

static void L3G4200D_LowLevel_Init(void);
static uint8_t L3G4200D_SendByte(uint8_t byte);

static void Delay_1ms( int nCnt_1ms );


void L3G4200D_Init(void)
{
  uint8_t ctrl = 0x00;  
  
  /* Configure the low level interface ---------------------------------------*/
  L3G4200D_LowLevel_Init();
  
	/* Required delay for the MEMS Accelerometer: Turn-on time = 3/Output data Rate
	                                                            = 3/100 = 30ms */
	Delay_1ms(100);

  	ctrl = 0x0F; //100Hz
	L3G4200D_Write(&ctrl, L3G4200D_CTRL_REG1_ADDR, 1);

	Delay_1ms(100);
}

void Delay_1ms( int nCnt_1ms )
{
    int nCnt;
          for(; nCnt_1ms != 0; nCnt_1ms--)
                    for(nCnt = 56580; nCnt != 0; nCnt--);
}


void L3G4200D_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    if(NumByteToWrite>0x01)
    {
        WriteAddr|=(uint8_t)MULTIPLEBYTE_CMD;
    }

    L3G4200D_CS_LOW();

    L3G4200D_SendByte(WriteAddr);
    while(NumByteToWrite>=0x01)
    {
        L3G4200D_SendByte(*pBuffer);
        NumByteToWrite--;
        pBuffer++;
    }

    L3G4200D_CS_HIGH();
}   // L3G4200D_Write



/**
  * @brief  Reads a block of data from the L3G4200D.
  * @param  pBuffer : pointer to the buffer that receives the data read from the L3G4200D.
  * @param  ReadAddr : L3G4200D's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the L3G4200D.
  * @retval None
  */
void L3G4200D_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  L3G4200D_CS_LOW();
  
  /* Send the Address of the indexed register */
  L3G4200D_SendByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to L3G4200D (Slave device) */
    *pBuffer = L3G4200D_SendByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  L3G4200D_CS_HIGH();
}


/**
  * @brief  Initializes the low level interface used to drive the L3G4200D
  * @param  None
  * @retval None
  */
static void L3G4200D_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //GPIO_OType_OD
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; //GPIO_PuPd_UP
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_SPI2); // SCK
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_SPI2); // MISO
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_SPI2); // MOSI


  /* SPI configuration -------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;// set to full duplex mode, seperate MOSI and MISO lines
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;// transmit in master mode, NSS pin has to be always high
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;// one packet of data is 8 bits wide
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;// clock is low when idle
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;// data sampled at first edge
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;         // set the NSS management to internal and pull internal NSS high
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;// SPI frequency is APB2 frequency / 4
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
  SPI_InitStructure.SPI_CRCPolynomial = 7;  // ??????????????????
  SPI_Init(L3G4200D_SPI, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI2, ENABLE);

  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;    // PINE1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(GPIOE, GPIO_Pin_1);
}

static uint8_t L3G4200D_SendByte(uint8_t byte)
{

  while (!SPI_I2S_GetFlagStatus(L3G4200D_SPI, SPI_I2S_FLAG_TXE));  
  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(L3G4200D_SPI, byte);

  while (!SPI_I2S_GetFlagStatus(L3G4200D_SPI, SPI_I2S_FLAG_RXNE));
   
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(L3G4200D_SPI);
}





