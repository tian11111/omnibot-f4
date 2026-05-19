/**
 * @file  stepper_interface.c
 * @brief Bottom-level GPIO + UART driver for two EMM42 stepper modules.
 *
 * Pin mapping (as per user schematic)
 * ------------------------------------
 * M1: EN=PD3  PUL=PB14  DIR=PG5  TX=PG14(USART6_TX)  RX=PG9(USART6_RX)
 * M2: EN=PB13 PUL=PB15  DIR=PC9  TX=PD5(USART2_TX)   RX=PD6(USART2_RX)
 *
 * Known conflicts resolved here:
 *   PG14, PG9   - were GPIO outputs in MX_GPIO_Init, now AF USART6.
 *   PB13        - was generic GPIO output, now M2_EN.
 *   PB14        - was generic GPIO output (also TIM1_CH2 PWM in original
 *                 CubeMX), now M1_PUL.
 *   PB15        - was generic GPIO output, now M2_PUL.
 */

#include "stepper_interface.h"
#include "usart.h"   /* huart2 */
#include <string.h>

/* ------------------------------------------------------------------ */
/* USART6 handle (M1 serial)                                          */
/* ------------------------------------------------------------------ */
UART_HandleTypeDef huart6;

static void MX_USART6_UART_Init(void)
{
    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 115200;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ------------------------------------------------------------------ */
/* Static helpers                                                      */
/* ------------------------------------------------------------------ */
static void StepperIF_ConfigureGpioPin(GPIO_TypeDef *port, uint16_t pin,
                                       uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = pin;
    gi.Mode  = mode;
    gi.Pull  = pull;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gi);
}

/* Pulse timing: ~10 us high at 168 MHz (conservative busy-wait) */
static void pulse_bang(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    for (volatile uint32_t i = 0; i < 420U; ++i) { __NOP(); }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void StepperIF_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    /* ---- Enable peripheral clocks ---- */
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();

    /* ---- Reconfigure PG14: GPIO -> USART6_TX (AF8) ---- */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14);
    gi.Pin       = GPIO_PIN_14;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &gi);

    /* ---- Reconfigure PG9: GPIO -> USART6_RX (AF8) ---- */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9);
    gi.Pin       = GPIO_PIN_9;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_PULLUP;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &gi);

    /* ---- Now safe to init USART6 peripheral ---- */
    /* NOTE: HAL_UART_Init calls HAL_UART_MspInit in usart.c.
     *       Since that function has no USART6 branch, the GPIO
     *       and NVIC for USART6 are configured manually here. */
    MX_USART6_UART_Init();

    /* Enable USART6 NVIC interrupt */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    /* ---- M1 GPIO: EN=PD3, PUL=PB14, DIR=PG5 ---- */
    StepperIF_ConfigureGpioPin(M1_EN_PORT, M1_EN_PIN,
                               GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(M1_EN_PORT, M1_EN_PIN, GPIO_PIN_SET);  /* default disabled */

    HAL_GPIO_DeInit(M1_PUL_PORT, M1_PUL_PIN);
    StepperIF_ConfigureGpioPin(M1_PUL_PORT, M1_PUL_PIN,
                               GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(M1_PUL_PORT, M1_PUL_PIN, GPIO_PIN_RESET);

    StepperIF_ConfigureGpioPin(M1_DIR_PORT, M1_DIR_PIN,
                               GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(M1_DIR_PORT, M1_DIR_PIN, GPIO_PIN_RESET);

    /* ---- M2 GPIO: EN=PB13, PUL=PB15, DIR=PC9 ---- */
    HAL_GPIO_DeInit(M2_EN_PORT, M2_EN_PIN);
    StepperIF_ConfigureGpioPin(M2_EN_PORT, M2_EN_PIN,
                               GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(M2_EN_PORT, M2_EN_PIN, GPIO_PIN_SET);

    HAL_GPIO_DeInit(M2_PUL_PORT, M2_PUL_PIN);
    StepperIF_ConfigureGpioPin(M2_PUL_PORT, M2_PUL_PIN,
                               GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(M2_PUL_PORT, M2_PUL_PIN, GPIO_PIN_RESET);

    StepperIF_ConfigureGpioPin(M2_DIR_PORT, M2_DIR_PIN,
                               GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(M2_DIR_PORT, M2_DIR_PIN, GPIO_PIN_RESET);

    /* NOTE: huart2 (USART2, PD5/PD6 115200 8N1) is already initialised
     *       by MX_USART2_UART_Init() in main.c before this call. */
}

/* ---- Enable / Disable ---- */
void StepperIF_M1_Enable(void)  { HAL_GPIO_WritePin(M1_EN_PORT, M1_EN_PIN, GPIO_PIN_RESET); }
void StepperIF_M1_Disable(void) { HAL_GPIO_WritePin(M1_EN_PORT, M1_EN_PIN, GPIO_PIN_SET);   }
void StepperIF_M2_Enable(void)  { HAL_GPIO_WritePin(M2_EN_PORT, M2_EN_PIN, GPIO_PIN_RESET); }
void StepperIF_M2_Disable(void) { HAL_GPIO_WritePin(M2_EN_PORT, M2_EN_PIN, GPIO_PIN_SET);   }

/* ---- Direction ---- */
void StepperIF_M1_SetDir(uint8_t forward)
{
    HAL_GPIO_WritePin(M1_DIR_PORT, M1_DIR_PIN,
                      forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void StepperIF_M2_SetDir(uint8_t forward)
{
    HAL_GPIO_WritePin(M2_DIR_PORT, M2_DIR_PIN,
                      forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---- Single step pulse ---- */
void StepperIF_M1_Pulse(void) { pulse_bang(M1_PUL_PORT, M1_PUL_PIN); }
void StepperIF_M2_Pulse(void) { pulse_bang(M2_PUL_PORT, M2_PUL_PIN); }

/* ---- UART transmit (blocking) ---- */
void StepperIF_M1_Transmit(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)data, len, 50U);
}

void StepperIF_M2_Transmit(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 50U);
}

/* ---- UART transmit (DMA) ---- */
void StepperIF_M1_TransmitDMA(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)data, len);
}

void StepperIF_M2_TransmitDMA(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, len);
}

