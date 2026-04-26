/**
 * @file motor_driver_emm42.c
 * @brief Emm42_V5.0 闭环步进驱动串口控制最小骨架实现。
 */

#include "motor_driver_emm42.h"
/* main.h 已通过 motor_driver_emm42.h 间接引入，
   stm32f1xx.h / stm32f1xx_hal.h / dma.h 均无需在此单独包含 */

/* -----------------------------------------------------------------------
 * DMA 发送完成回调（DMA 启用后须在 HAL_UART_TxCpltCallback 里调用）
 * ----------------------------------------------------------------------- */
void Emm42_TxCpltCallback(Emm42_Handle *handle, UART_HandleTypeDef *huart)
{
    if (handle != NULL && handle->huart == huart)
    {
        handle->tx_busy = false;
    }
}

/* -----------------------------------------------------------------------
 * 内部函数：打包并发送一帧命令
 * ----------------------------------------------------------------------- */
static Emm42_Status Emm42_SendSimpleCommand(Emm42_Handle *handle,
                                            uint8_t func,
                                            const uint8_t *payload,
                                            uint8_t length)
{
    if (handle == NULL || handle->huart == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    if (handle->checksum_mode != EMM42_CHECKSUM_FIXED_0x6B)
    {
        return EMM42_ERR_PARAM;
    }

    const uint8_t max_payload = (uint8_t)(sizeof(handle->tx_buf) - 3U);
    if (length > max_payload)
    {
        return EMM42_ERR_PARAM;
    }

    if (handle->tx_busy)
    {
        return EMM42_ERR_BUSY;
    }

    /* 打包：地址 + 功能码 + 数据 + 固定校验 0x6B */
    handle->tx_buf[0] = handle->id_addr;
    handle->tx_buf[1] = func;
    for (uint8_t i = 0U; i < length; ++i)
    {
        handle->tx_buf[2U + i] = (payload != NULL) ? payload[i] : 0U;
    }
    handle->tx_buf[2U + length] = 0x6BU;

    const uint16_t total_len = (uint16_t)(3U + length);

    /* 动态超时：每字节约 1ms（适用 9600bps 以上），加 5ms 裕量 */
    const uint32_t timeout_ms = (uint32_t)total_len + 5U;
    if (HAL_UART_Transmit(handle->huart, handle->tx_buf, total_len, timeout_ms) != HAL_OK)
    {
        return EMM42_ERR_HW;
    }

    return EMM42_OK;
}

/* -----------------------------------------------------------------------
 * 公开 API 实现
 * ----------------------------------------------------------------------- */

Emm42_Status Emm42_Init(Emm42_Handle *handle,
                        UART_HandleTypeDef *huart,
                        uint8_t id_addr)
{
    if (handle == NULL || huart == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    if (id_addr == 0U)
    {
        return EMM42_ERR_PARAM;
    }

    handle->huart         = huart;
    handle->id_addr       = id_addr;
    handle->checksum_mode = EMM42_CHECKSUM_FIXED_0x6B;
    handle->tx_busy       = false;

    return EMM42_OK;
}

Emm42_Status Emm42_SetChecksumMode(Emm42_Handle *handle,
                                   Emm42_ChecksumMode mode)
{
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    handle->checksum_mode = mode;
    return EMM42_OK;
}

Emm42_Status Emm42_Enable(Emm42_Handle *handle,
                          bool enable,
                          bool sync)
{
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    uint8_t payload[3U];
    payload[0] = 0xABU;
    payload[1] = enable ? 0x01U : 0x00U;
    payload[2] = sync   ? 0x01U : 0x00U;

    return Emm42_SendSimpleCommand(handle, 0xF3U, payload, (uint8_t)sizeof(payload));
}

Emm42_Status Emm42_SetSpeed(Emm42_Handle *handle,
                            bool ccw,
                            uint16_t rpm,
                            uint8_t accel,
                            bool sync)
{
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    uint8_t payload[5U];
    payload[0] = ccw  ? 0x01U : 0x00U;
    payload[1] = (uint8_t)(rpm >> 8U);
    payload[2] = (uint8_t)(rpm & 0xFFU);
    payload[3] = accel;
    payload[4] = sync ? 0x01U : 0x00U;

    return Emm42_SendSimpleCommand(handle, 0xF6U, payload, (uint8_t)sizeof(payload));
}

Emm42_Status Emm42_MoveTo(Emm42_Handle *handle,
                          bool ccw,
                          uint16_t rpm,
                          uint8_t accel,
                          uint32_t pulses,
                          bool use_absolute,
                          bool sync)
{
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    uint8_t payload[10U];
    payload[0] = ccw          ? 0x01U : 0x00U;
    payload[1] = (uint8_t)(rpm >> 8U);
    payload[2] = (uint8_t)(rpm & 0xFFU);
    payload[3] = accel;
    payload[4] = (uint8_t)(pulses >> 24U);
    payload[5] = (uint8_t)((pulses >> 16U) & 0xFFU);
    payload[6] = (uint8_t)((pulses >>  8U) & 0xFFU);
    payload[7] = (uint8_t)(pulses          & 0xFFU);
    payload[8] = use_absolute ? 0x01U : 0x00U;
    payload[9] = sync         ? 0x01U : 0x00U;

    return Emm42_SendSimpleCommand(handle, 0xFDU, payload, (uint8_t)sizeof(payload));
}

Emm42_Status Emm42_StopNow(Emm42_Handle *handle,
                           bool sync)
{
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    uint8_t payload[2U];
    payload[0] = 0x98U;
    payload[1] = sync ? 0x01U : 0x00U;

    return Emm42_SendSimpleCommand(handle, 0xFEU, payload, (uint8_t)sizeof(payload));
}

Emm42_Status Emm42_SyncStart(Emm42_Handle *handle)
{
    if (handle == NULL || handle->huart == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    /* static const：防止 DMA 启用后读已释放的栈内存 */
    static const uint8_t sync_frame[4U] = {0x00U, 0xFFU, 0x66U, 0x6BU};

    if (HAL_UART_Transmit(handle->huart, (uint8_t *)sync_frame, 4U, 4U + 5U) != HAL_OK)
    {
        return EMM42_ERR_HW;
    }

    return EMM42_OK;
}
