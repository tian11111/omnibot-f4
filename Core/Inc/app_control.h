#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "main.h"
#include "motor_driver_emm42.h"

/* 定义USE_MECANUM启用麦轮控制 */
#define USE_MECANUM

#ifdef USE_MECANUM
#include "motor_driver_dc4ch.h"
#endif

/* Gripper stepper (EMM42) always available */
void Gripper_Init(void);
void Gripper_SetSpeed(int16_t x_speed, int16_t y_speed);
void Gripper_StopAll(void);

#ifdef USE_MECANUM
/* Mecanum DC drive */
void Mecanum_Init(void);
void Mecanum_SetMotion(int16_t vx, int16_t vy, int16_t wz);
void Mecanum_StopAll(void);
#endif

void App_ControlTask(void);
void App_AutoPlotTask(void);

/* 自动绘图控制 */
extern uint8_t g_auto_plot_enable;
extern uint16_t g_auto_plot_interval_ms;

#endif

