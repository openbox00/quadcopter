/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery_lis3dsh.h"



__IO uint32_t  LIS3DSHTimeout = LIS3DSH_FLAG_TIMEOUT;   

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

static void LIS3DSH_LowLevel_Init(void);
static uint8_t LIS3DSH_SendByte(uint8_t byte);

/**
  * @brief  Set LIS3DSH Initialization.
  * @param  LIS3DSH_Config_Struct: pointer to a LIS3DSH_Config_TypeDef structure 
  *         that contains the configuration setting for the LIS3DSH.
  * @retval None
  */
void LIS3DSH_Init(LIS3DSH_InitTypeDef *LIS3DSH_InitStruct)
{
	uint8_t ctrl = 0x00;	
  	uint32_t i=0;		//simple index for software delay
  
	/* Configure the low level interface ---------------------------------------*/
	LIS3DSH_LowLevel_Init();
  
	/* Required delay for the MEMS Accelerometer: Turn-on time = 3/Output data Rate
	                                                            = 3/100 = 30ms */
  /* 0x20 */  
  /* 
  ctrl=(uint8_t) (LIS3DSH_InitStruct->CR4_Odr|LIS3DSH_InitStruct->CR4_Bdu|
     LIS3DSH_InitStruct->CR4_Zen|LIS3DSH_InitStruct->CR4_Yen|
     LIS3DSH_InitStruct->CR4_Xen);
*/
 //ctrl = 0x47;  
  ctrl = 0x67; //100Hz
  LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG4_ADDR, 1);

#if 0
  /* 0x21 */  
	ctrl=LIS3DSH_InitStruct->SM1_Hysteresis|LIS3DSH_InitStruct->SM1_Pin|LIS3DSH_InitStruct->SM1_Enable;
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG1_ADDR, 1);
	
  /* 0x22 */  
	ctrl=LIS3DSH_InitStruct->SM2_Hysteresis|LIS3DSH_InitStruct->SM2_Pin|LIS3DSH_InitStruct->SM2_Enable;
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG2_ADDR, 1);
	
  /* 0x23 */  
	ctrl=LIS3DSH_InitStruct->CR3_Dren|LIS3DSH_InitStruct->CR3_Iea|LIS3DSH_InitStruct->CR3_Iel|
		 LIS3DSH_InitStruct->CR3_Int2En|LIS3DSH_InitStruct->CR3_Int1En|LIS3DSH_InitStruct->CR3_Vfilt|
		 LIS3DSH_InitStruct->CR3_Strt;
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG3_ADDR, 1);
	
  /* 0x24 */  
	ctrl=LIS3DSH_InitStruct->CR5_Bw|LIS3DSH_InitStruct->CR5_Fscale|
		 LIS3DSH_InitStruct->CR5_St|LIS3DSH_InitStruct->CR5_Sim;
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG5_ADDR, 1);
	
  /* 0x25 */  
	ctrl=LIS3DSH_InitStruct->CR6_Boot|LIS3DSH_InitStruct->CR6_FifoEn|
		 LIS3DSH_InitStruct->CR6_WtmEn|LIS3DSH_InitStruct->CR6_AddInc|
		 LIS3DSH_InitStruct->CR6_P1Empty|LIS3DSH_InitStruct->CR6_P1Wtm|
		 LIS3DSH_InitStruct->CR6_P1OverRun|LIS3DSH_InitStruct->CR6_P2Boot;
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG6_ADDR, 1);
#endif
}


void LIS3DSH_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    if(NumByteToWrite>0x01)
    {
        WriteAddr|=(uint8_t)MULTIPLEBYTE_CMD;
    }

    LIS3DSH_CS_LOW();

    LIS3DSH_SendByte(WriteAddr);
    while(NumByteToWrite>=0x01)
    {
        LIS3DSH_SendByte(*pBuffer);
        NumByteToWrite--;
        pBuffer++;
    }

    LIS3DSH_CS_HIGH();
}   // LIS3DSH_Write

/**
  * @brief  Reads a block of data from the LIS3DSH.
  * @param  pBuffer : pointer to the buffer that receives the data read from the LIS3DSH.
  * @param  ReadAddr : LIS3DSH's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the LIS3DSH.
  * @retval None
  */
void LIS3DSH_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
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
  LIS3DSH_CS_LOW();
  
  /* Send the Address of the indexed register */
  LIS3DSH_SendByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS3DSH (Slave device) */
    *pBuffer = LIS3DSH_SendByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  LIS3DSH_CS_HIGH();
}


/**
  * @brief  Initializes the low level interface used to drive the LIS3DSH
  * @param  None
  * @retval None
  */
static void LIS3DSH_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(LIS3DSH_SPI_CLK, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHB1PeriphClockCmd(LIS3DSH_SPI_SCK_GPIO_CLK | LIS3DSH_SPI_MISO_GPIO_CLK | LIS3DSH_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable CS  GPIO clock */
  RCC_AHB1PeriphClockCmd(LIS3DSH_SPI_CS_GPIO_CLK, ENABLE);
  
  /* Enable INT1 GPIO clock */
  RCC_AHB1PeriphClockCmd(LIS3DSH_SPI_INT1_GPIO_CLK, ENABLE);
  
  /* Enable INT2 GPIO clock */
  RCC_AHB1PeriphClockCmd(LIS3DSH_SPI_INT2_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(LIS3DSH_SPI_SCK_GPIO_PORT, LIS3DSH_SPI_SCK_SOURCE, LIS3DSH_SPI_SCK_AF);
  GPIO_PinAFConfig(LIS3DSH_SPI_MISO_GPIO_PORT, LIS3DSH_SPI_MISO_SOURCE, LIS3DSH_SPI_MISO_AF);
  GPIO_PinAFConfig(LIS3DSH_SPI_MOSI_GPIO_PORT, LIS3DSH_SPI_MOSI_SOURCE, LIS3DSH_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = LIS3DSH_SPI_SCK_PIN;
  GPIO_Init(LIS3DSH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  LIS3DSH_SPI_MOSI_PIN;
  GPIO_Init(LIS3DSH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = LIS3DSH_SPI_MISO_PIN;
  GPIO_Init(LIS3DSH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(LIS3DSH_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(LIS3DSH_SPI, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(LIS3DSH_SPI, ENABLE);

  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = LIS3DSH_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LIS3DSH_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(LIS3DSH_SPI_CS_GPIO_PORT, LIS3DSH_SPI_CS_PIN);
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructure.GPIO_Pin = LIS3DSH_SPI_INT1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(LIS3DSH_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = LIS3DSH_SPI_INT2_PIN;
  GPIO_Init(LIS3DSH_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
}


/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
static uint8_t LIS3DSH_SendByte(uint8_t byte)
{
  /* Loop while DR register in not emplty */
  LIS3DSHTimeout = LIS3DSH_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(LIS3DSH_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((LIS3DSHTimeout--) == 0) return LIS3DSH_TIMEOUT_UserCallback();
  }
  
  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(LIS3DSH_SPI, byte);
  
  /* Wait to receive a Byte */
  LIS3DSHTimeout = LIS3DSH_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(LIS3DSH_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((LIS3DSHTimeout--) == 0) return LIS3DSH_TIMEOUT_UserCallback();
  }
  
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(LIS3DSH_SPI);
}


#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS3DSH_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {   
  }
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */



