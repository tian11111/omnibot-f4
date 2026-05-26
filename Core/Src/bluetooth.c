#include "bluetooth.h"
#include "motor_closedloop.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart3;

uint8_t BT_RxFlag = 0;
//接收完成标志位
char BT_RxPacket[BT_RX_PACKET_MAX_LEN];
//接收数据缓存

static uint8_t bt_rx_byte = 0;
// 接收状态机
static uint8_t bt_rx_state = 0;
// 接收索引
static uint8_t bt_rx_index = 0;
// 接收数据长度

void Bluetooth_Init(void)
{
    memset(BT_RxPacket, 0, sizeof(BT_RxPacket));
    bt_rx_state = 0;
    bt_rx_index = 0;
    BT_RxFlag = 0;
}

void Bluetooth_StartReceiveIT(void)
{
    HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (bt_rx_state == 0)
        {
            if (bt_rx_byte == '[' && BT_RxFlag == 0)
            {
                bt_rx_state = 1;
                bt_rx_index = 0;
                memset(BT_RxPacket, 0, sizeof(BT_RxPacket));
            }
        }
        else if (bt_rx_state == 1)
        {
            if (bt_rx_byte == ']')
            {
                bt_rx_state = 0;
                BT_RxPacket[bt_rx_index] = '\0';
                BT_RxFlag = 1;
            }
            else
            {
                if (bt_rx_index < BT_RX_PACKET_MAX_LEN - 1)
                {
                    BT_RxPacket[bt_rx_index++] = (char)bt_rx_byte;
                }
                else
                {
                    // 长度超了，直接丢包复位
                    bt_rx_state = 0;
                    bt_rx_index = 0;
                    memset(BT_RxPacket, 0, sizeof(BT_RxPacket));
                }
            }
        }

        HAL_UART_Receive_IT(&huart3, &bt_rx_byte, 1);
    }
}

void Bluetooth_SendString(const char *str)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 100);
}

void Bluetooth_SendMotorStatus(void)
{
    char tx_buf[BT_TX_PACKET_MAX_LEN];
    int32_t speed0 = MotorClosedLoop_GetCurrentSpeed(0);
    int32_t speed1 = MotorClosedLoop_GetCurrentSpeed(1);
    int32_t speed2 = MotorClosedLoop_GetCurrentSpeed(2);
    int32_t speed3 = MotorClosedLoop_GetCurrentSpeed(3);

    snprintf(tx_buf, BT_TX_PACKET_MAX_LEN,
             "[s,%ld,%ld,%ld,%ld]\r\n",
             speed0, speed1, speed2, speed3);

    Bluetooth_SendString(tx_buf);
}
