#ifndef MOTOR_CLOSEDLOOP_H
#define MOTOR_CLOSEDLOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 编码器定时器定义 */
#define ENCODER_TIM_FL  &htim2  /* 前轮左/右: PA0/PA1 */
#define ENCODER_TIM_FR  &htim3  /* 前轮右/左: PA7/PA6 */
#define ENCODER_TIM_RL  &htim4  /* 后轮左/右: PB7/PB6 */
#define ENCODER_TIM_RR  &htim8  /* 后轮右/左: PC7/PC6 */

/* 电机索引定义 */
#define MOTOR_FL  0  /* 前轮左 */
#define MOTOR_FR  1  /* 前轮右 */
#define MOTOR_RL  2  /* 后轮左 */
#define MOTOR_RR  3  /* 后轮右 */

/* PID参数结构体 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} PID_Controller;

/* 电机闭环控制结构体 */
typedef struct {
    TIM_HandleTypeDef *encoder_tim;
    int32_t encoder_count;
    int32_t target_speed;
    int32_t current_speed;
    PID_Controller pid;
    int16_t motor_output;
} MotorClosedLoop;

/* 函数声明 */
void MotorClosedLoop_Init(void);
void MotorClosedLoop_Start(void);
void MotorClosedLoop_Stop(void);
void MotorClosedLoop_SetTargetSpeed(uint8_t motor_idx, int32_t target_speed);
void MotorClosedLoop_Update(void);
int32_t MotorClosedLoop_GetEncoderCount(uint8_t motor_idx);
int32_t MotorClosedLoop_GetCurrentSpeed(uint8_t motor_idx);
void MotorClosedLoop_ResetEncoder(uint8_t motor_idx);
void MotorClosedLoop_ResetAllEncoders(void);
void MotorClosedLoop_SetPIDParams(uint8_t motor_idx, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CLOSEDLOOP_H */
