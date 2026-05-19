#ifndef STEPPER_INTERFACE_H
#define STEPPER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Hardware pin mapping (as per user schematic)
 *
 * M1: EN=PD3  PUL=PB14  DIR=PG5  TX=PG14(USART6_TX)  RX=PG9(USART6_RX)
 * M2: EN=PB13 PUL=PB15  DIR=PC9  TX=PD5(USART2_TX)   RX=PD6(USART2_RX)
 *
 * NOTE: M1 uses USART6 (new), M2 reuses existing USART2 (115200 8N1).
 *       PG14 and PG9 will be reconfigured from GPIO output to AF USART6.
 *       PB13/PB14/PB15 will be reconfigured from generic GPIO to
 *       dedicated EN/PUL/DIR functions.
 * ----------------------------------------------------------------------- */

/* M1 GPIO */
#define M1_EN_PORT   GPIOD
#define M1_EN_PIN    GPIO_PIN_3
#define M1_PUL_PORT  GPIOB
#define M1_PUL_PIN   GPIO_PIN_14
#define M1_DIR_PORT  GPIOG
#define M1_DIR_PIN   GPIO_PIN_5

/* M2 GPIO */
#define M2_EN_PORT   GPIOB
#define M2_EN_PIN    GPIO_PIN_13
#define M2_PUL_PORT  GPIOB
#define M2_PUL_PIN   GPIO_PIN_15
#define M2_DIR_PORT  GPIOC
#define M2_DIR_PIN   GPIO_PIN_9

/* UART handles (declared extern, defined in this module) */
extern UART_HandleTypeDef huart6;
/* huart2 is already defined in usart.c */

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/** @brief One-time init: GPIO + USART6 + reconfigure conflicting pins. */
void StepperIF_Init(void);

/** @brief Set EN pin (LOW = enabled on most drivers, adjust if active-high). */
void StepperIF_M1_Enable(void);
void StepperIF_M1_Disable(void);
void StepperIF_M2_Enable(void);
void StepperIF_M2_Disable(void);

/** @brief DIR pin control. */
void StepperIF_M1_SetDir(uint8_t forward);
void StepperIF_M2_SetDir(uint8_t forward);

/** @brief Generate a single pulse on PUL pin (blocking ~10us). */
void StepperIF_M1_Pulse(void);
void StepperIF_M2_Pulse(void);

/** @brief Send raw bytes to stepper driver via UART (blocking). */
void StepperIF_M1_Transmit(const uint8_t *data, uint16_t len);
void StepperIF_M2_Transmit(const uint8_t *data, uint16_t len);

/** @brief Send raw bytes via UART with DMA (non-blocking). */
void StepperIF_M1_TransmitDMA(const uint8_t *data, uint16_t len);
void StepperIF_M2_TransmitDMA(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_INTERFACE_H */

