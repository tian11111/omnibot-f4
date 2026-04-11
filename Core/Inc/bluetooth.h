#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

#define BT_RX_PACKET_MAX_LEN    64

extern uint8_t BT_RxFlag;
extern char BT_RxPacket[BT_RX_PACKET_MAX_LEN];

void Bluetooth_Init(void);
void Bluetooth_StartReceiveIT(void);

#endif
