/**
 * @file  motor_driver_X42S.c
 * @brief X42S 步进电机升降控制
 *
 * 基于已验证的 X_V2 8 字节协议，阻塞发送。
 * M1: USART6 (PG14/PG9), EN=PD3 (LOW 使能), 地址 1。
 */

#include "motor_driver_X42S.h"
#include "usart.h"
#include <stdlib.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/* ---- 协议常量 ---- */
#define X42S_FC_ENABLE    0xF3U
#define X42S_FC_VELOCITY  0xF6U
#define X42S_FC_STOP      0xFEU
#define X42S_FOOTER       0x6BU

/* ===================================================================
 * 底层协议（已验证可用）
 * =================================================================== */

/* ---- 速度模式：已验证，不改 ---- */
void X_V2_Vel_Control(UART_HandleTypeDef *huart,uint8_t addr, uint8_t dir, uint16_t acc, float vel, bool snF)
{
  uint8_t cmd[16] = {0}; uint16_t v = 0;

  // 将速度放大10倍发送过去
	#define ABS(x) (x)>0?(x):-(x)

//  v = (uint16_t)ABS(vel * 10.0f);
	v=(uint16_t)ABS(vel);

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 符号（方向）
 // 加速度(RPM/s)
//  cmd[3] =  (uint8_t)(acc >> 8);
//  cmd[4] =  (uint8_t)(v >> 0);        	// 速度(RPM)
//  cmd[5] =  (uint8_t)(v >> 8);
	cmd[3] =  (uint8_t)(v >> 8);         // 速度(RPM)高8位 (Speed_H)
  cmd[4] =  (uint8_t)(v & 0xFF);       // 速度(RPM)低8位 (Speed_L)
  cmd[5] =  acc;                       // 加速度 (Acc，1字节，0-255)
	
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节

  // 发送命令
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 8,HAL_MAX_DELAY);
}


/* ---- 立即停止 ---- */
static void X42S_SendStop(UART_HandleTypeDef *huart,uint8_t addr)
{
    X_V2_Vel_Control(huart,addr, 1, 0, 0, false);
}

void MotorDriverX42S_Serial_Init(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);   /* EN = PD3, LOW 使能 */
    HAL_Delay(10);
}

void MotorDriverX42S_SetDualSpeed(int16_t x_speed, int16_t y_speed)
{
    /* ---------------- 控制电机 X (串口6) ---------------- */
    if (x_speed >  (int16_t)X42S_MAX_VEL) x_speed =  (int16_t)X42S_MAX_VEL;
    if (x_speed < -(int16_t)X42S_MAX_VEL) x_speed = -(int16_t)X42S_MAX_VEL;

    if (x_speed == 0) {
        X42S_SendStop(&huart6, 1);
    } else {
        uint8_t dir_x = (x_speed > 0) ? 0U : 1U;
        X_V2_Vel_Control(&huart6, 1, dir_x, 30, (float)abs(x_speed), false);
    }

    /* ---------------- 控制电机 Y (串口2) ---------------- */
    if (y_speed >  (int16_t)X42S_MAX_VEL) y_speed =  (int16_t)X42S_MAX_VEL;
    if (y_speed < -(int16_t)X42S_MAX_VEL) y_speed = -(int16_t)X42S_MAX_VEL;

    if (y_speed == 0) {
        /* 注意：由于是在两个独立的串口上，地址通常都是默认的 1 (X42S_MOTOR_ADDR) */
        X42S_SendStop(&huart2, 1);
    } else {
        uint8_t dir_y = (y_speed > 0) ? 0U : 1U;
        X_V2_Vel_Control(&huart2, 1, dir_y, 30, (float)abs(y_speed), false);
    }
}
