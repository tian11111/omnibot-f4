#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "stm32f4xx_hal.h"

#define SOFT_I2C_SCL_PORT    GPIOC
#define SOFT_I2C_SCL_PIN     GPIO_PIN_1
#define SOFT_I2C_SDA_PORT    GPIOC
#define SOFT_I2C_SDA_PIN     GPIO_PIN_0

void Soft_I2C_Init(void);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
uint8_t Soft_I2C_WaitAck(void);
void Soft_I2C_WriteByte(uint8_t data);

#endif
