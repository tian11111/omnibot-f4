/**
  * @file  motor_closedloop.c
  * @brief 四轮闭环控制模块，使用编码器反馈实现速度闭环
  *
  * 编码器定时器配置：
  *   前轮左/右: TIM2 (PA0/PA1)
  *   前轮右/左: TIM3 (PA7/PA6)
  *   后轮左/右: TIM4 (PB7/PB6)
  *   后轮右/左: TIM8 (PC7/PC6)
  */

#include "motor_closedloop.h"
#include "motor_driver_dc4ch.h"
#include "tim.h"
#include <math.h>

/* 闭环控制实例 */
static MotorClosedLoop g_motors[DC4_MOTOR_COUNT];

/* 编码器定时器句柄数组 */
static TIM_HandleTypeDef* g_encoder_timers[DC4_MOTOR_COUNT] = {
    ENCODER_TIM_FL,
    ENCODER_TIM_FR,
    ENCODER_TIM_RL,
    ENCODER_TIM_RR
};

/* 编码器方向反向表：1表示反向，0表示正向 */
static const uint8_t g_encoder_invert[DC4_MOTOR_COUNT] = {
    0,  /* 前左 */
    0,  /* 前右 */
    0,  /* 后左 */
    0   /* 后右 */
};

/* 速度缩放因子：将目标速度(-100~100)转换为编码器计数单位 */
/* 需要根据实际电机和编码器调整此值 */
#define SPEED_SCALE_FACTOR  1.5f

/* PID控制器初始化 */
static void PID_Init(PID_Controller *pid)
{
    pid->kp = 0.5f;
    pid->ki = 0.3f;
    pid->kd = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = -100.0f;
    pid->output_max = 100.0f;
}

/* PID计算 */
static float PID_Compute(PID_Controller *pid, float error)
{
    /* 比例项 */
    float p_term = pid->kp * error;
    
    /* 积分项（带限幅） */
    pid->integral += error;
    if (pid->integral > 1000.0f) pid->integral = 1000.0f;
    if (pid->integral < -1000.0f) pid->integral = -1000.0f;
    float i_term = pid->ki * pid->integral;
    
    /* 微分项 */
    float d_term = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;
    
    /* 输出限幅 */
    float output = p_term + i_term + d_term;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}

/* 初始化闭环控制模块 */
void MotorClosedLoop_Init(void)
{
    for (uint8_t i = 0; i < DC4_MOTOR_COUNT; i++) {
        g_motors[i].encoder_tim = g_encoder_timers[i];
        g_motors[i].encoder_count = 0;
        g_motors[i].target_speed = 0;
        g_motors[i].current_speed = 0;
        g_motors[i].motor_output = 0;
        PID_Init(&g_motors[i].pid);
    }
}

/* 启动编码器 */
void MotorClosedLoop_Start(void)
{
    for (uint8_t i = 0; i < DC4_MOTOR_COUNT; i++) {
        HAL_TIM_Encoder_Start(g_motors[i].encoder_tim, TIM_CHANNEL_ALL);
    }
}

/* 停止编码器 */
void MotorClosedLoop_Stop(void)
{
    for (uint8_t i = 0; i < DC4_MOTOR_COUNT; i++) {
        HAL_TIM_Encoder_Stop(g_motors[i].encoder_tim, TIM_CHANNEL_ALL);
    }
}

/* 设置目标速度 */
void MotorClosedLoop_SetTargetSpeed(uint8_t motor_idx, int32_t target_speed)
{
    if (motor_idx >= DC4_MOTOR_COUNT) return;
    g_motors[motor_idx].target_speed = target_speed;
}

/* 获取编码器计数 */
int32_t MotorClosedLoop_GetEncoderCount(uint8_t motor_idx)
{
    if (motor_idx >= DC4_MOTOR_COUNT) return 0;
    return (int32_t)__HAL_TIM_GET_COUNTER(g_motors[motor_idx].encoder_tim);
}

/* 获取当前速度 */
int32_t MotorClosedLoop_GetCurrentSpeed(uint8_t motor_idx)
{
    if (motor_idx >= DC4_MOTOR_COUNT) return 0;
    return g_motors[motor_idx].current_speed;
}

/* 重置编码器计数 */
void MotorClosedLoop_ResetEncoder(uint8_t motor_idx)
{
    if (motor_idx >= DC4_MOTOR_COUNT) return;
    __HAL_TIM_SET_COUNTER(g_motors[motor_idx].encoder_tim, 0);
    g_motors[motor_idx].encoder_count = 0;
}

/* 重置所有编码器 */
void MotorClosedLoop_ResetAllEncoders(void)
{
    for (uint8_t i = 0; i < DC4_MOTOR_COUNT; i++) {
        MotorClosedLoop_ResetEncoder(i);
    }
}

/* 设置PID参数 */
void MotorClosedLoop_SetPIDParams(uint8_t motor_idx, float kp, float ki, float kd)
{
    if (motor_idx >= DC4_MOTOR_COUNT) return;
    g_motors[motor_idx].pid.kp = kp;
    g_motors[motor_idx].pid.ki = ki;
    g_motors[motor_idx].pid.kd = kd;
}

/* 更新闭环控制（需要定期调用，建议10ms周期） */
void MotorClosedLoop_Update(void)
{
    for (uint8_t i = 0; i < DC4_MOTOR_COUNT; i++) {
        /* 读出当前计数值，直接强转为有符号16位（自动处理了正反转溢出） */
        int16_t raw_speed = (int16_t)__HAL_TIM_GET_COUNTER(g_motors[i].encoder_tim);
        
        /* 立即清零定时器，为下一个周期做准备 */
        __HAL_TIM_SET_COUNTER(g_motors[i].encoder_tim, 0);
        
        /* 应用编码器方向反向 */
        if (g_encoder_invert[i]) {
            raw_speed = -raw_speed;
        }
        
        /* 使用raw_speed作为当前速度 */
        g_motors[i].current_speed = raw_speed;
        if (g_motors[i].target_speed == 0) {
					g_motors[i].pid.integral = 0.0f;
					g_motors[i].pid.prev_error = 0.0f;
					g_motors[i].motor_output = 0;
					DC4_Motor_SetSignedSpeed(i, 0);
					continue;
				}
        /* 将目标速度缩放到编码器计数单位 */
        float scaled_target = (float)g_motors[i].target_speed * SPEED_SCALE_FACTOR;
        
        /* 计算速度误差 */
        float error = scaled_target - (float)g_motors[i].current_speed;
        
        /* PID计算 */
        float pid_output = PID_Compute(&g_motors[i].pid, error);
        
        /* 转换为电机输出 */
        g_motors[i].motor_output = (int16_t)pid_output;
        
        /* 设置电机速度 */
        DC4_Motor_SetSignedSpeed(i, g_motors[i].motor_output);
    }
}

