#include "stdio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"

#define WHOAMI							0x0f
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

uint8_t I2C_write(uint8_t devAddr, uint8_t regAddr, uint8_t val);
uint8_t I2C_readreg(uint8_t devAddr, uint8_t regAddr);
void init_I2C1(void);
void sensor_ayarla();
