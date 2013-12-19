#include "stdio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"

#define CTRL_REG1                       0x20
#define CTRL_REG2                       0x21
#define CTRL_REG3                       0x22
#define CTRL_REG4                       0x23
#define CTRL_REG5                       0x24
#define STATUS_REG                      0x27
#define OUT_X_L                         0x28
#define OUT_X_H                         0x29
#define OUT_Y_L                         0x2A
#define OUT_Y_H                         0x2B
#define OUT_Z_L                         0x2C
#define OUT_Z_H                         0x2D
#define L3G4200D_ADDR                   (105<<1)

uint8_t i2cdata,a1,a2;
int16_t gyroX, gyroY, gyroZ;

uint8_t I2C_write(uint8_t devAddr, uint8_t regAddr, uint8_t val);
uint8_t I2C_readreg(uint8_t devAddr, uint8_t regAddr);

void init_I2C1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL pini 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA pini

    I2C_InitStruct.I2C_ClockSpeed = 100000;    //i2c hızı normal modda                
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);

    I2C_Cmd(I2C1, ENABLE);
}


uint8_t I2C_write(uint8_t devAddr, uint8_t regAddr, uint8_t val)
{
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY ));

        I2C_GenerateSTART(I2C1, ENABLE);

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT ));

        I2C_Send7bitAddress(I2C1, devAddr, I2C_Direction_Transmitter );

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ));

        I2C_SendData(I2C1, regAddr);

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING ));

        I2C_SendData(I2C1, val);

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING ));

        I2C_GenerateSTOP(I2C1, ENABLE);
}

uint8_t I2C_readreg(uint8_t devAddr, uint8_t regAddr)
{
        uint8_t reg;
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY ));

        I2C_GenerateSTART(I2C1, ENABLE);

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT ));

        I2C_Send7bitAddress(I2C1, devAddr, I2C_Direction_Transmitter );

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ));

        I2C_SendData(I2C1, regAddr);

        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF ) == RESET);

        I2C_GenerateSTOP(I2C1, ENABLE);

        I2C_GenerateSTART(I2C1, ENABLE);

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT ));

        I2C_Send7bitAddress(I2C1, devAddr, I2C_Direction_Receiver );

        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR ) == RESET);

    I2C_AcknowledgeConfig(I2C1, DISABLE);

    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED ));

    reg = I2C_ReceiveData(I2C1);

        I2C_GenerateSTOP(I2C1, ENABLE);

        return reg;
}

void sensor_ayarla()
{
    I2C_write(L3G4200D_ADDR, CTRL_REG1, 0x0F); // X,Y,Z eksenlerini aktif et ve 100Hz data çıkış frekansı 
    I2C_write(L3G4200D_ADDR, CTRL_REG2, 0x00); 
    I2C_write(L3G4200D_ADDR, CTRL_REG3, 0x08); //eğer istenilirse sensörün int2 (data hazır) interuptı kullanılabilir
    I2C_write(L3G4200D_ADDR, CTRL_REG4, 0x30); //çıkış değeri  2000d/s olacak şekilde ayarla
    I2C_write(L3G4200D_ADDR, CTRL_REG5, 0x00);
}

/*int main(void)
{

    SystemInit();
    init_I2C1();
    sensor_ayarla();

    while (1){

    if((I2C_readreg(L3G4200D_ADDR,STATUS_REG)&0x08)==0x08) // eğer yeni data kullanılmaya hazırsa dataları oku
    {
        a1=I2C_readreg(L3G4200D_ADDR,OUT_X_L);
        a2=I2C_readreg(L3G4200D_ADDR,OUT_X_H);
        gyroX=((a2<<8) | a1);

        a1=I2C_readreg(L3G4200D_ADDR,OUT_Y_L);
        a2=I2C_readreg(L3G4200D_ADDR,OUT_Y_H);
        gyroY=((a2<<8) | a1);

        a1=I2C_readreg(L3G4200D_ADDR,OUT_Z_L);
        a2=I2C_readreg(L3G4200D_ADDR,OUT_Z_H);
        gyroZ=((a2<<8) | a1);
    }
    }
}*/
