/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery_l3g4200d.h"



__IO uint32_t  L3G4200DTimeout = L3G4200D_FLAG_TIMEOUT;   

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

static void L3G4200D_LowLevel_Init(void);
static uint8_t L3G4200D_SendByte(uint8_t byte);


void L3G4200D_Init(L3G4200D_InitTypeDef *L3G4200D_InitStruct)
{
	uint8_t ctrl = 0x00;	
  
	/* Configure the low level interface ---------------------------------------*/
	L3G4200D_LowLevel_Init();
  
  ctrl = 0x17; //100Hz
  L3G4200D_Write(&ctrl, L3G4200D_CTRL_REG1_ADDR, 1);
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
  RCC_APB1PeriphClockCmd(L3G4200D_SPI_CLK, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHB1PeriphClockCmd(L3G4200D_SPI_SCK_GPIO_CLK | L3G4200D_SPI_MISO_GPIO_CLK | L3G4200D_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable CS  GPIO clock */
//  RCC_AHB1PeriphClockCmd(L3G4200D_SPI_CS_GPIO_CLK, ENABLE);
  
  /* Enable INT1 GPIO clock */
  //RCC_AHB1PeriphClockCmd(L3G4200D_SPI_INT1_GPIO_CLK, ENABLE);
  
  /* Enable INT2 GPIO clock */
  //RCC_AHB1PeriphClockCmd(L3G4200D_SPI_INT2_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(L3G4200D_SPI_SCK_GPIO_PORT, L3G4200D_SPI_SCK_SOURCE, L3G4200D_SPI_SCK_AF);
  GPIO_PinAFConfig(L3G4200D_SPI_MISO_GPIO_PORT, L3G4200D_SPI_MISO_SOURCE, L3G4200D_SPI_MISO_AF);
  GPIO_PinAFConfig(L3G4200D_SPI_MOSI_GPIO_PORT, L3G4200D_SPI_MOSI_SOURCE, L3G4200D_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(L3G4200D_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
  GPIO_Init(L3G4200D_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_Init(L3G4200D_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI2);  ///// APB2 42M need to change detail
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;// set to full duplex mode, seperate MOSI and MISO lines
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;// transmit in master mode, NSS pin has to be always high
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;// one packet of data is 8 bits wide
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;// clock is low when idle
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;// data sampled at first edge
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;         // set the NSS management to internal and pull internal NSS high
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;// SPI frequency is APB2 frequency / 4
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
  SPI_InitStructure.SPI_CRCPolynomial = 7;  // ??????????????????
  SPI_Init(L3G4200D_SPI, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI2, ENABLE);


/*?????????????????????????????????????????????????????????????????????????????*/
  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = L3G4200D_SPI_CS_PIN;    // PINE1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(L3G4200D_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(L3G4200D_SPI_CS_GPIO_PORT, L3G4200D_SPI_CS_PIN);
  #if 0
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructure.GPIO_Pin = L3G4200D_SPI_INT1_PIN;  // pin0????????????
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(L3G4200D_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = L3G4200D_SPI_INT2_PIN;  //pin 1???????????
  GPIO_Init(L3G4200D_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
  /*?????????????????????????????????????????????????????????????????????????????*/
  #endif
}

static uint8_t L3G4200D_SendByte(uint8_t byte)
{
  /* Loop while DR register in not emplty */
  L3G4200DTimeout = L3G4200D_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(L3G4200D_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((L3G4200DTimeout--) == 0) return L3G4200D_TIMEOUT_UserCallback();
  }
  
  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(L3G4200D_SPI, byte);
  
  /* Wait to receive a Byte */
  L3G4200DTimeout = L3G4200D_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(L3G4200D_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((L3G4200DTimeout--) == 0) return L3G4200D_TIMEOUT_UserCallback();
  }
  
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(L3G4200D_SPI);
}


#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
uint32_t L3G4200D_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {   
  }
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */



