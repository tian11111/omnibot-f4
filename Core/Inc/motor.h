#ifndef __motor_H
#define __motor_H

#include "main.h"

// 电机编号
typedef enum
{
    MOTOR_FL = 0,   // Front Left  前左
    MOTOR_FR,       // Front Right 前右
    MOTOR_RL,       // Rear Left   后左
    MOTOR_RR        // Rear Right  后右
} MotorId_t;

// 初始化
void Motor_Init(void);

// 单个电机设置速度
// speed范围：-1000 ~ 1000
// 正负表示方向，绝对值表示占空比
void Motor_SetSpeed(MotorId_t id, int16_t speed);

// 全部停止
void Motor_StopAll(void);

// 麦克纳姆轮速度控制
// vx：前后，正数前进
// vy：左右，正数右移
// wz：旋转，正数顺时针/逆时针你可按实际调整
// 输入范围建议：-1000 ~ 1000
void Mecanum_SetSpeed(int16_t vx, int16_t vy, int16_t wz);

#endif
