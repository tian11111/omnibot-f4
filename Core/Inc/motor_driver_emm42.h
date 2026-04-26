/**
 * @file motor_driver_emm42.h
 * @brief Emm42_V5.0 闭环步进驱动串口控制接口。
 */

#ifndef LOWER_MOTOR_DRIVER_EMM42_H
#define LOWER_MOTOR_DRIVER_EMM42_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"       /* 包含全部 HAL 头文件，含 UART_HandleTypeDef */
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * 枚举定义
 * ----------------------------------------------------------------------- */

typedef enum
{
    EMM42_CHECKSUM_FIXED_0x6B = 0U,
    EMM42_CHECKSUM_XOR,
    EMM42_CHECKSUM_CRC8,
} Emm42_ChecksumMode;

typedef enum
{
    EMM42_OK = 0,
    EMM42_ERR_PARAM,
    EMM42_ERR_TIMEOUT,
    EMM42_ERR_BUSY,
    EMM42_ERR_HW,
} Emm42_Status;

/* -----------------------------------------------------------------------
 * 句柄结构体
 * ----------------------------------------------------------------------- */

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t             id_addr;
    Emm42_ChecksumMode  checksum_mode;
    uint8_t             tx_buf[16U];   /* DMA 发送缓冲，生命周期与句柄相同 */
    volatile bool       tx_busy;       /* DMA 传输进行中标志，TC 回调中清零 */
} Emm42_Handle;

/* -----------------------------------------------------------------------
 * 函数声明
 * ----------------------------------------------------------------------- */

Emm42_Status Emm42_Init(Emm42_Handle *handle,
                        UART_HandleTypeDef *huart,
                        uint8_t id_addr);

Emm42_Status Emm42_SetChecksumMode(Emm42_Handle *handle,
                                   Emm42_ChecksumMode mode);

Emm42_Status Emm42_Enable(Emm42_Handle *handle,
                          bool enable,
                          bool sync);

Emm42_Status Emm42_SetSpeed(Emm42_Handle *handle,
                            bool ccw,
                            uint16_t rpm,
                            uint8_t accel,
                            bool sync);

/* 注意：原名 Emm42_MoveRelative / Emm42_MoveT0（含笔误）均已统一为 Emm42_MoveTo */
Emm42_Status Emm42_MoveTo(Emm42_Handle *handle,
                          bool ccw,
                          uint16_t rpm,
                          uint8_t accel,
                          uint32_t pulses,
                          bool use_absolute,
                          bool sync);

Emm42_Status Emm42_StopNow(Emm42_Handle *handle,
                           bool sync);

Emm42_Status Emm42_SyncStart(Emm42_Handle *handle);

/**
 * @brief DMA 发送完成回调，须在 HAL_UART_TxCpltCallback 里调用。
 *
 * 示例（stm32f1xx_it.c 或 main.c）：
 *   void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
 *       extern Emm42_Handle motor1;
 *       Emm42_TxCpltCallback(&motor1, huart);
 *   }
 */
void Emm42_TxCpltCallback(Emm42_Handle *handle, UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* LOWER_MOTOR_DRIVER_EMM42_H */
