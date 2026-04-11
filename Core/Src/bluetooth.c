#include "bluetooth.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

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
    HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
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

        HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
    }
}