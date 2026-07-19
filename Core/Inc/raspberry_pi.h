#ifndef __RASPBERRY_PI_H
#define __RASPBERRY_PI_H

#include "main.h"

#define RPI_RX_PACKET_MAX_LEN    64

extern uint8_t g_rpi_data_ready;
extern char    g_rpi_color[16];
extern uint16_t g_rpi_code1;
extern uint16_t g_rpi_code2;
extern uint8_t s_rpi_rx_byte;

void RaspberryPi_Init(void);
void RaspberryPi_StartReceiveIT(void);
void RaspberryPi_RxCallback(void);
void RaspberryPi_DisplayUpdate(void);
void RaspberryPi_SendReady(void);

#endif /* __RASPBERRY_PI_H */
