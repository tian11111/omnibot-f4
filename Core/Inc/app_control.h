#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "main.h"

void Mecanum_Init(void);
void Mecanum_SetVelocity(int16_t vx, int16_t vy, int16_t wz);
void App_ControlTask(void);

#endif
