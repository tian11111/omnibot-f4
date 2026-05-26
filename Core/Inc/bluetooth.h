#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

#define BT_RX_PACKET_MAX_LEN    64
#define BT_TX_PACKET_MAX_LEN    128

extern uint8_t BT_RxFlag;
extern char BT_RxPacket[BT_RX_PACKET_MAX_LEN];

void Bluetooth_Init(void);
void Bluetooth_StartReceiveIT(void);
void Bluetooth_SendString(const char *str);
void Bluetooth_SendMotorStatus(void);

#endif
