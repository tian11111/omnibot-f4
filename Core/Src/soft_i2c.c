#include "soft_i2c.h"

#define OLED_W_SCL(x)  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, (GPIO_PinState)(x))
#define OLED_W_SDA(x)  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, (GPIO_PinState)(x))

static void I2C_Delay(void)
{
    volatile uint32_t i = 50;
    while(i--);
}

void Soft_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = SOFT_I2C_SCL_PIN | SOFT_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    OLED_W_SCL(1);
    OLED_W_SDA(1);
    I2C_Delay();
}

void Soft_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    I2C_Delay();
    OLED_W_SDA(0);
    I2C_Delay();
    OLED_W_SCL(0);
    I2C_Delay();
}

void Soft_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    I2C_Delay();
    OLED_W_SDA(1);
    I2C_Delay();
}

void Soft_I2C_WriteByte(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA(!!(data & (0x80 >> i)));
        I2C_Delay();
        OLED_W_SCL(1);
        I2C_Delay();
        OLED_W_SCL(0);
        I2C_Delay();
    }
    OLED_W_SDA(1);
    I2C_Delay();
    OLED_W_SCL(1);
    I2C_Delay();
    OLED_W_SCL(0);
    I2C_Delay();
}

uint8_t Soft_I2C_WaitAck(void)
{
    return 0;
}
