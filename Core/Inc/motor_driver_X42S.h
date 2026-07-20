#ifndef MOTOR_DRIVER_X42S_H
#define MOTOR_DRIVER_X42S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- 电机参数 ---- */
#define X42S_MOTOR1_ADDR      1U        /* M1 地址（USART2），默认 ID=1 */
#define X42S_MOTOR2_ADDR      1U        /* M2 地址（USART6），默认 ID=2 */

#define X42S_MAX_VEL         800.0f    /* 最大速度 RPM */

#define X42S_MOTOR_X_DIR_INVERT  0U   /* USART6 (PG14/PG9) 电机方向取反：1=取反；当前接线正向 */
#define X42S_MOTOR_Y_DIR_INVERT  0U   /* USART2 电机方向取反：1=取反 */

void X_V2_Vel_Control(UART_HandleTypeDef *huart,uint8_t addr, uint8_t dir, uint16_t acc, float vel, bool snF);
void MotorDriverX42S_SetDualSpeed(int16_t x_speed, int16_t y_speed); 
void MotorDriverX42S_Serial_Init(void);

void MotorDriverX42S_ControlTask(void);           /* 蓝牙控制任务（主循环调用） */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_X42S_H */
