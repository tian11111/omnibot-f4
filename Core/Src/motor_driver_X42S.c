/**
 * @file  motor_driver_X42S.c
 * @brief X42S step motor lift control
 *
 * X_V2 8-byte protocol, DMA non-blocking parallel TX.
 * M1(X): USART6 (PG14/PG9), EN=PD3 (LOW enable), addr 1.
 * M2(Y): USART2 (PD5/PD6), addr 1.
 */

#include "motor_driver_X42S.h"
#include "usart.h"
#include <stdlib.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/* ---- protocol constants ---- */
#define X42S_FC_ENABLE    0xF3U
#define X42S_FC_VELOCITY  0xF6U
#define X42S_FC_STOP      0xFEU
#define X42S_FOOTER       0x6BU

/* DMA TX buffers; must remain valid while DMA in flight, so global */
static uint8_t g_x42s_tx_buf_x[8];
static uint8_t g_x42s_tx_buf_y[8];

/* Build velocity command frame into cmd[8] */
static void X42S_BuildVelCmd(uint8_t *cmd, uint8_t addr, uint8_t dir,
                             uint16_t acc, float vel, bool snF)
{
    uint16_t v = (uint16_t)((vel >= 0.0f) ? vel : -vel);

    cmd[0] = addr;
    cmd[1] = 0xF6U;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(v >> 8);
    cmd[4] = (uint8_t)(v & 0xFFU);
    cmd[5] = (uint8_t)acc;
    cmd[6] = snF;
    cmd[7] = 0x6BU;
}

/* Backward-compatible blocking send */
void X_V2_Vel_Control(UART_HandleTypeDef *huart, uint8_t addr,
                      uint8_t dir, uint16_t acc, float vel, bool snF)
{
    uint8_t cmd[8];
    X42S_BuildVelCmd(cmd, addr, dir, acc, vel, snF);
    HAL_UART_Transmit(huart, cmd, 8, HAL_MAX_DELAY);
}

/* DMA non-blocking send; abort previous TX if still busy */
static void X42S_DMASend(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size)
{
    if (huart->gState != HAL_UART_STATE_READY)
    {
        HAL_UART_AbortTransmit_IT(huart);
    }
    (void)HAL_UART_Transmit_DMA(huart, buf, size);
}

void MotorDriverX42S_Serial_Init(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);   /* EN=PD3, LOW enable */
    HAL_Delay(10);
}

void MotorDriverX42S_SetDualSpeed(int16_t x_speed, int16_t y_speed)
{
    /* ---- limit ---- */
    if (x_speed >  (int16_t)X42S_MAX_VEL) x_speed =  (int16_t)X42S_MAX_VEL;
    if (x_speed < -(int16_t)X42S_MAX_VEL) x_speed = -(int16_t)X42S_MAX_VEL;
    if (y_speed >  (int16_t)X42S_MAX_VEL) y_speed =  (int16_t)X42S_MAX_VEL;
    if (y_speed < -(int16_t)X42S_MAX_VEL) y_speed = -(int16_t)X42S_MAX_VEL;

    /* ---- build X frame (USART6) ---- */
    if (x_speed == 0) {
        X42S_BuildVelCmd(g_x42s_tx_buf_x, 1, 1, 0, 0.0f, false);
    } else {
        uint8_t dir_x = (x_speed > 0) ? 0U : 1U;
        if (X42S_MOTOR_X_DIR_INVERT) dir_x ^= 1U;
        X42S_BuildVelCmd(g_x42s_tx_buf_x, 1, dir_x, 30, (float)abs(x_speed), false);
    }

    /* ---- build Y frame (USART2) ---- */
    if (y_speed == 0) {
        X42S_BuildVelCmd(g_x42s_tx_buf_y, 1, 1, 0, 0.0f, false);
    } else {
        uint8_t dir_y = (y_speed > 0) ? 0U : 1U;
        if (X42S_MOTOR_Y_DIR_INVERT) dir_y ^= 1U;
        X42S_BuildVelCmd(g_x42s_tx_buf_y, 1, dir_y, 30, (float)abs(y_speed), false);
    }

    /* ---- fire both UARTs in parallel via DMA ---- */
    X42S_DMASend(&huart6, g_x42s_tx_buf_x, 8);
    X42S_DMASend(&huart2, g_x42s_tx_buf_y, 8);
}

void MotorDriverX42S_ControlTask(void)
{
    /* no-op: control driven by App_ControlTask -> SetDualSpeed */
}
