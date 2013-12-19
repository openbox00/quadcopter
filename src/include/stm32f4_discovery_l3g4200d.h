#ifndef __STM32F4_DISCOVERY_L3G4200D_H
#define __STM32F4_DISCOVERY_L3G4200D_H

#include "stm32f4xx.h"

#define L3G4200D_WHOAMI_REG_ADDR       0x0f

#define L3G4200D_CTRL_REG1_ADDR        0x20
#define L3G4200D_CTRL_REG2_ADDR        0x21
#define L3G4200D_CTRL_REG3_ADDR        0x22
#define L3G4200D_CTRL_REG4_ADDR        0x23
#define L3G4200D_CTRL_REG5_ADDR        0x24

#define L3G4200D_REFERENCE_REG_ADDR    0x25

#define L3G4200D_OUT_TEMP_REG5_ADDR    0x26
#define L3G4200D_STATUS_REG_ADDR       0x27

#define L3G4200D_OUT_X_L_REG_ADDR      0x28
#define L3G4200D_OUT_X_H_REG_ADDR      0x29
#define L3G4200D_OUT_Y_L_REG_ADDR      0x2a
#define L3G4200D_OUT_Y_H_REG_ADDR      0x2b
#define L3G4200D_OUT_Z_L_REG_ADDR      0x2c
#define L3G4200D_OUT_Z_H_REG_ADDR      0x2d

#define L3G4200D_FIFO_CTRL_REG_ADDR    0x2e
#define L3G4200D_FIFO_SRC_REG_ADDR     0x0e

#define L3G4200D_INT1_CFG_REG_ADDR     0x30
#define L3G4200D_INT1_SRC_REG_ADDR     0x31

#define L3G4200D_INT1_THS_XH_REG_ADDR  0x32
#define L3G4200D_INT1_THS_XL_REG_ADDR  0x32

#define L3G4200D_INT1_THS_YH_REG_ADDR  0x32
#define L3G4200D_INT1_THS_YL_REG_ADDR  0x32

#define L3G4200D_INT1_THS_ZH_REG_ADDR  0x32
#define L3G4200D_INT1_THS_ZL_REG_ADDR  0x32

#define L3G4200D_INT1_DURATION_REG_ADDR  0x38



/*************        SPI        *****************/
#define L3G4200D_SPI                       SPI2
#define L3G4200D_SPI_CLK                   RCC_APB1Periph_SPI2

#define L3G4200D_SPI_SCK_PIN               GPIO_Pin_13
#define L3G4200D_SPI_SCK_GPIO_PORT         GPIOB
#define L3G4200D_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define L3G4200D_SPI_SCK_SOURCE            GPIO_PinSource13
#define L3G4200D_SPI_SCK_AF                GPIO_AF_SPI2

#define L3G4200D_SPI_MISO_PIN              GPIO_Pin_14
#define L3G4200D_SPI_MISO_GPIO_PORT        GPIOB
#define L3G4200D_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define L3G4200D_SPI_MISO_SOURCE           GPIO_PinSource14
#define L3G4200D_SPI_MISO_AF               GPIO_AF_SPI2

#define L3G4200D_SPI_MOSI_PIN              GPIO_Pin_15
#define L3G4200D_SPI_MOSI_GPIO_PORT        GPIOB
#define L3G4200D_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define L3G4200D_SPI_MOSI_SOURCE           GPIO_PinSource15
#define L3G4200D_SPI_MOSI_AF               GPIO_AF_SPI2

/****************************************??????????????????**********/
#define L3G4200D_SPI_CS_PIN                GPIO_Pin_1
#define L3G4200D_SPI_CS_GPIO_PORT          GPIOE
#define L3G4200D_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOE

#define L3G4200D_SPI_INT1_PIN              GPIO_Pin_0
#define L3G4200D_SPI_INT1_GPIO_PORT        GPIOE
#define L3G4200D_SPI_INT1_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define L3G4200D_SPI_INT1_EXTI_LINE        EXTI_Line0
#define L3G4200D_SPI_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define L3G4200D_SPI_INT1_EXTI_PIN_SOURCE  EXTI_PinSource0
#define L3G4200D_SPI_INT1_EXTI_IRQn        EXTI0_IRQn

#define L3G4200D_SPI_INT2_PIN              GPIO_Pin_1
#define L3G4200D_SPI_INT2_GPIO_PORT        GPIOE
#define L3G4200D_SPI_INT2_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define L3G4200D_SPI_INT2_EXTI_LINE        EXTI_Line1
#define L3G4200D_SPI_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define L3G4200D_SPI_INT2_EXTI_PIN_SOURCE  EXTI_PinSource1
#define L3G4200D_SPI_INT2_EXTI_IRQn        EXTI1_IRQn
/**************************************************/


#define L3G4200D_CS_LOW()    GPIO_ResetBits(L3G4200D_SPI_CS_GPIO_PORT, L3G4200D_SPI_CS_PIN)
#define L3G4200D_CS_HIGH()   GPIO_SetBits(L3G4200D_SPI_CS_GPIO_PORT, L3G4200D_SPI_CS_PIN)

#define L3G4200D_FLAG_TIMEOUT    ((uint32_t)0x1000)

typedef struct
{
    // **** Control Register 1 ****
    uint8_t CR1_Dr;
    uint8_t CR1_Bw;
    uint8_t CR1_Pd;
    uint8_t CR1_Zen;
    uint8_t CR1_Yen;
    uint8_t CR1_Xen;        
    // **** END OF Control Register 1 ****

    // **** Control Register 2 ****
    uint8_t CR2_Hpm;
    uint8_t CR2_Hpcf;
    // **** END OF Control Register 2 ****

    // **** Control Register 3 ****
    uint8_t CR3_I1_int1;
    uint8_t CR3_I1_Boot;
    uint8_t CR3_H_Lactive;
    uint8_t CR3_PP_OD;
    uint8_t CR3_I2_DRDY;
    uint8_t CR3_I2_WTM;
    uint8_t CR3_I2_ORun;
    uint8_t CR3_Empty;    
    // **** END OF Control Register 3

    // **** Control Register 4 ****
    uint8_t CR4_BDU;
    uint8_t CR4_BLE;
    uint8_t CR4_FS;
    uint8_t CR4_ST;
    uint8_t CR4_SIM;
    // **** END OF Control Register 4

    // **** Control Register 5 ****
    uint8_t CR5_BOOT;
    uint8_t CR5_FIFO_EN;
    uint8_t CR5_INT1_Sel1;
    uint8_t CR5_Out_Sel1;
    // **** END OF Control Register 5
} L3G4200D_InitTypeDef;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} gyro_vector;


/** @defgroup STM32F4_DISCOVERY_L3G4200DH_Exported_Functions
  * @{
  */ 
void L3G4200D_Init(L3G4200D_InitTypeDef *L3G4200D_InitStruct);
void L3G4200D_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void L3G4200D_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);

uint32_t L3G4200D_TIMEOUT_UserCallback(void);

#endif /* __STM32F4_DISCOVERY_L3G4200DH_H */

