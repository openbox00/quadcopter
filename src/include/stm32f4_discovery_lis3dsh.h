#ifndef __STM32F4_DISCOVERY_LIS3DS_H
#define __STM32F4_DISCOVERY_LIS3DS_H

#include "stm32f4xx.h"

#define LIS3DSH_CTRL_REG1_ADDR  0x21
#define LIS3DSH_CTRL_REG2_ADDR  0x22
#define LIS3DSH_CTRL_REG3_ADDR  0x23
#define LIS3DSH_CTRL_REG4_ADDR  0x20
#define LIS3DSH_CTRL_REG5_ADDR  0x24
#define LIS3DSH_CTRL_REG6_ADDR  0x25

#define LIS3DSH_INFO1_REG_ADDR  0x0d
#define LIS3DSH_INFO2_REG_ADDR  0x0e
#define LIS3DSH_WHOAMI_REG_ADDR 0x0f

#define LIS3DSH_STATUS_REG_ADDR 0x27

#define LIS3DSH_TEMPERATURE_REG_ADDR    0x0c

#define LIS3DSH_OUT_X_L_REG_ADDR    0x28
#define LIS3DSH_OUT_X_H_REG_ADDR    0x29
#define LIS3DSH_OUT_Y_L_REG_ADDR    0x2a
#define LIS3DSH_OUT_Y_H_REG_ADDR    0x2b
#define LIS3DSH_OUT_Z_L_REG_ADDR    0x2c
#define LIS3DSH_OUT_Z_H_REG_ADDR    0x2d

#define LIS3DSH_FLAG_ZXYOR   ((uint8_t)0x00)
#define LIS3DSH_FLAG_ZOR     ((uint8_t)0x01)
#define LIS3DSH_FLAG_YOR     ((uint8_t)0x02)
#define LIS3DSH_FLAG_XOR     ((uint8_t)0x03)
#define LIS3DSH_FLAG_ZXYDA   ((uint8_t)0x04)
#define LIS3DSH_FLAG_ZDA     ((uint8_t)0x05)
#define LIS3DSH_FLAG_YDA     ((uint8_t)0x06)
#define LIS3DSH_FLAG_XDA     ((uint8_t)0x07)

#define DEVICE_ID   ((uint8_t)0x3f)

#define LIS3DSH_SM1_INT_TO_PIN_INT1 ((uint8_t)0x00)
#define LIS3DSH_SM1_INT_TO_PIN_INT2 ((uint8_t)0x01)

#define LIS3DSH_SM1_DISABLE ((uint8_t)0x00)
#define LIS3DSH_SM1_ENABLE  ((uint8_t)0x01)

#define LIS3DSH_SM2_INT_TO_PIN_INT1 ((uint8_t)0x00)
#define LIS3DSH_SM2_INT_TO_PIN_INT2 ((uint8_t)0x01)

#define LIS3DSH_SM2_DISABLE ((uint8_t)0x00)
#define LIS3DSH_SM2_ENABLE  ((uint8_t)0x01)

#define LIS3DSH_CR3_DREN_TO_INT1_DISABLE    ((uint8_t)0x00)
#define LIS3DSH_CR3_DREN_TO_INT1_ENABLE     ((uint8_t)0x01)

#define LIS3DSH_CR3_IEA_ACTIVE_LOW  ((uint8_t)0x00)
#define LIS3DSH_CR3_IEA_ACTIVE_HIGH ((uint8_t)0x01)

#define LIS3DSH_CR3_IEL_LATCHED ((uint8_t)0x00)
#define LIS3DSH_CR3_IEL_PULSED  ((uint8_t)0x01)

#define LIS3DSH_CR3_INT2_DISABLED   ((uint8_t)0x00)
#define LIS3DSH_CR3_INT2_ENABLED    ((uint8_t)0x01)

#define LIS3DSH_CR3_INT1_DISABLED   ((uint8_t)0x00)
#define LIS3DSH_CR3_INT1_ENABLED    ((uint8_t)0x01)

#define LIS3DSH_CR3_VFILT_DISABLED  ((uint8_t)0x00)
#define LIS3DSH_CR3_VFILT_ENABLED   ((uint8_t)0x01)

#define LIS3DSH_CR3_NO_SOFT_RESET   ((uint8_t)0x00)
#define LIS3DSH_CR3_SOFT_RESET      ((uint8_t)0x01)

#define LIS3DSH_CR4_ODR_POWER_DOWN  ((uint8_t)0x00)
#define LIS3DSH_CR4_ODR_3f125HZ     ((uint8_t)0x01)
#define LIS3DSH_CR4_ODR_6f25HZ      ((uint8_t)0x02)
#define LIS3DSH_CR4_ODR_12f5HZ      ((uint8_t)0x03)
#define LIS3DSH_CR4_ODR_25HZ        ((uint8_t)0x04)
#define LIS3DSH_CR4_ODR_50HZ        ((uint8_t)0x05)
#define LIS3DSH_CR4_ODR_100HZ       ((uint8_t)0x06)
#define LIS3DSH_CR4_ODR_400HZ       ((uint8_t)0x07)
#define LIS3DSH_CR4_ODR_800HZ       ((uint8_t)0x08)
#define LIS3DSH_CR4_ODR_1600HZ      ((uint8_t)0x09)

#define LIS3DSH_CR4_BDU_DISABLED    ((uint8_t)0x00)
#define LIS3DSH_CR4_BDU_ENABLED     ((uint8_t)0x01)

#define LIS3DSH_CR4_Z_AXIS_DISABLED ((uint8_t)0x00)
#define LIS3DSH_CR4_Z_AXIS_ENABLED  ((uint8_t)0x01)

#define LIS3DSH_CR4_X_AXIS_DISABLED ((uint8_t)0x00)
#define LIS3DSH_CR4_X_AXIS_ENABLED  ((uint8_t)0x01)

#define LIS3DSH_CR4_Y_AXIS_DISABLED ((uint8_t)0x00)
#define LIS3DSH_CR4_Y_AXIS_ENABLED  ((uint8_t)0x01)

#define LIS3DSH_CR5_BW_800HZ    ((uint8_t)0x00)
#define LIS3DSH_CR5_BW_400HZ    ((uint8_t)0x01)
#define LIS3DSH_CR5_BW_200HZ    ((uint8_t)0x02)
#define LIS3DSH_CR5_BW_50HZ     ((uint8_t)0x03)

#define LIS3DSH_CR5_FSCALE_2G   ((uint8_t)0x00)
#define LIS3DSH_CR5_FSCALE_4G   ((uint8_t)0x01)
#define LIS3DSH_CR5_FSCALE_6G   ((uint8_t)0x02)
#define LIS3DSH_CR5_FSCALE_8G   ((uint8_t)0x03)
#define LIS3DSH_CR5_FSCALE_16G  ((uint8_t)0x04)

#define LIS3DSH_CR5_ST_DISABLE      ((uint8_t)0x00)
#define LIS3DSH_CR5_ST_POSITIVE     ((uint8_t)0x01)
#define LIS3DSH_CR5_ST_NEGATIVE     ((uint8_t)0x02)
#define LIS3DSH_CR5_ST_NOT_ALLOWED  ((uint8_t)0x03)

#define LIS3DSH_CR5_MODE_4_WIRE_INTERFACE   ((uint8_t)0x00)
#define LIS3DSH_CR5_MODE_3_WIRE_INTERFACE   ((uint8_t)0x01)

#define LIS3DSH_CR6_FORCE_REBOOT_DISABLE    ((uint8_t)0x00)
#define LIS3DSH_CR6_FORCE_REBOOT_ENABLE     ((uint8_t)0x01)

#define LIS3DSH_CR6_FIFO_DISABLED   ((uint8_t)0x00)
#define LIS3DSH_CR6_FIFO_ENABLED    ((uint8_t)0x01)

#define LIS3DSH_CR6_WTM_DISABLED    ((uint8_t)0x00)
#define LIS3DSH_CR6_WTM_ENABLED     ((uint8_t)0x01)

#define LIS3DSH_CR6_ADDINC_DISABLED ((uint8_t)0x00)
#define LIS3DSH_CR6_ADDINC_ENABLED  ((uint8_t)0x01)

#define LIS3DSH_CR6_FIFO_EMPTY_TO_INT1_DISABLED ((uint8_t)0x00)
#define LIS3DSH_CR6_FIFO_EMPTY_TO_INT1_ENABLED  ((uint8_t)0x01)

#define LIS3DSH_CR6_FIFO_WTM_TO_INT1_DISABLED   ((uint8_t)0x00)
#define LIS3DSH_CR6_FIFO_WTM_TO_INT1_ENABLED    ((uint8_t)0x01)

#define LIS3DSH_CR6_FIFO_OVERRUN_TO_INT1_DISABLED   ((uint8_t)0x00)
#define LIS3DSH_CR6_FIFO_OVERRUN_TO_INT1_ENABLED    ((uint8_t)0x01)

#define LIS3DSH_CR6_BOOT_TO_INT2_DISABLED   ((uint8_t)0x00)
#define LIS3DSH_CR6_BOOT_TO_INT2_ENABLED    ((uint8_t)0x01)

#define LIS3DSH_SPI                       GPIO_PinAFConfig
#define LIS3DSH_SPI_CLK                   RCC_APB2Periph_SPI1

#define LIS3DSH_SPI_SCK_PIN               GPIO_Pin_5
#define LIS3DSH_SPI_SCK_GPIO_PORT         GPIOA
#define LIS3DSH_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define LIS3DSH_SPI_SCK_SOURCE            GPIO_PinSource5
#define LIS3DSH_SPI_SCK_AF                GPIO_AF_SPI1

#define LIS3DSH_SPI_MISO_PIN              GPIO_Pin_6
#define LIS3DSH_SPI_MISO_GPIO_PORT        GPIOA
#define LIS3DSH_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define LIS3DSH_SPI_MISO_SOURCE           GPIO_PinSource6
#define LIS3DSH_SPI_MISO_AF               GPIO_AF_SPI1

#define LIS3DSH_SPI_MOSI_PIN              GPIO_Pin_7
#define LIS3DSH_SPI_MOSI_GPIO_PORT        GPIOA
#define LIS3DSH_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define LIS3DSH_SPI_MOSI_SOURCE           GPIO_PinSource7
#define LIS3DSH_SPI_MOSI_AF               GPIO_AF_SPI1

#define LIS3DSH_SPI_CS_PIN                GPIO_Pin_3
#define LIS3DSH_SPI_CS_GPIO_PORT          GPIOE
#define LIS3DSH_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOE

#define LIS3DSH_SPI_INT1_PIN              GPIO_Pin_0
#define LIS3DSH_SPI_INT1_GPIO_PORT        GPIOE
#define LIS3DSH_SPI_INT1_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define LIS3DSH_SPI_INT1_EXTI_LINE        EXTI_Line0
#define LIS3DSH_SPI_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define LIS3DSH_SPI_INT1_EXTI_PIN_SOURCE  EXTI_PinSource0
#define LIS3DSH_SPI_INT1_EXTI_IRQn        EXTI0_IRQn

#define LIS3DSH_SPI_INT2_PIN              GPIO_Pin_1
#define LIS3DSH_SPI_INT2_GPIO_PORT        GPIOE
#define LIS3DSH_SPI_INT2_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define LIS3DSH_SPI_INT2_EXTI_LINE        EXTI_Line1
#define LIS3DSH_SPI_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define LIS3DSH_SPI_INT2_EXTI_PIN_SOURCE  EXTI_PinSource1
#define LIS3DSH_SPI_INT2_EXTI_IRQn        EXTI1_IRQn

#define LIS3DSH_CS_LOW()    GPIO_ResetBits(LIS3DSH_SPI_CS_GPIO_PORT, LIS3DSH_SPI_CS_PIN)
#define LIS3DSH_CS_HIGH()   GPIO_SetBits(LIS3DSH_SPI_CS_GPIO_PORT, LIS3DSH_SPI_CS_PIN)

#define LIS3DSH_FLAG_TIMEOUT    ((uint32_t)0x1000)

typedef struct
{
    // **** Control Register 1 ****
    uint8_t SM1_Hysteresis;
    uint8_t SM1_Pin;
    uint8_t SM1_Enable;
    // **** END OF Control Register 1 ****

    // **** Control Register 2 ****
    uint8_t SM2_Hysteresis;
    uint8_t SM2_Pin;
    uint8_t SM2_Enable;
    // **** END OF Control Register 2 ****

    // **** Control Register 3 ****
    uint8_t CR3_Dren;
    uint8_t CR3_Iea;
    uint8_t CR3_Iel;
    uint8_t CR3_Int2En;
    uint8_t CR3_Int1En;
    uint8_t CR3_Vfilt;
    uint8_t CR3_Strt;
    // **** END OF Control Register 3

    // **** Control Register 4 ****
    uint8_t CR4_Odr;
    uint8_t CR4_Bdu;
    uint8_t CR4_Zen;
    uint8_t CR4_Yen;
    uint8_t CR4_Xen;
    // **** END OF Control Register 4

    // **** Control Register 5 ****
    uint8_t CR5_Bw;
    uint8_t CR5_Fscale;
    uint8_t CR5_St;
    uint8_t CR5_Sim;
    // **** END OF Control Register 5

    // **** Control Register 6 ****
    uint8_t CR6_Boot;
    uint8_t CR6_FifoEn;
    uint8_t CR6_WtmEn;
    uint8_t CR6_AddInc;
    uint8_t CR6_P1Empty;
    uint8_t CR6_P1Wtm;
    uint8_t CR6_P1OverRun;
    uint8_t CR6_P2Boot;
    // **** END OF Control Register 6
} LIS3DSH_InitTypeDef;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} accel_vector;


/** @defgroup STM32F4_DISCOVERY_LIS3DSH_Exported_Functions
  * @{
  */ 
void LIS3DSH_Init(LIS3DSH_InitTypeDef *LIS3DSH_InitStruct);
void LIS3DSH_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void LIS3DSH_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);

/* USER Callbacks: This is function for which prototype only is declared in
   MEMS accelerometre driver and that should be implemented into user applicaiton. */  
/* LIS3DSH_TIMEOUT_UserCallback() function is called whenever a timeout condition 
   occure during communication (waiting transmit data register empty flag(TXE)
   or waiting receive data register is not empty flag (RXNE)).
   You can use the default timeout callback implementation by uncommenting the 
   define USE_DEFAULT_TIMEOUT_CALLBACK in stm32f4_discovery_LIS3DSH.h file.
   Typically the user implementation of this callback should reset MEMS peripheral
   and re-initialize communication or in worst case reset all the application. */
uint32_t LIS3DSH_TIMEOUT_UserCallback(void);

#endif /* __STM32F4_DISCOVERY_LIS3DSH_H */

