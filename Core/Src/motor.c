#include "motor.h"

extern TIM_HandleTypeDef htim1;

// PWM最大计数值（ARR）
#define MOTOR_PWM_MAX    1000

// 限幅宏
#define ABS(x)           ((x) > 0 ? (x) : -(x))

static int16_t limit_int16(int16_t val, int16_t min, int16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}



// 前左 FL
#define FL_IN1_GPIO_Port     GPIOB
#define FL_IN1_Pin           GPIO_PIN_12
#define FL_IN2_GPIO_Port     GPIOB
#define FL_IN2_Pin           GPIO_PIN_13

// 前右 FR
#define FR_IN1_GPIO_Port     GPIOG
#define FR_IN1_Pin           GPIO_PIN_5
#define FR_IN2_GPIO_Port     GPIOG
#define FR_IN2_Pin           GPIO_PIN_6

// 后左 RL
#define RL_IN1_GPIO_Port     GPIOE
#define RL_IN1_Pin           GPIO_PIN_2
#define RL_IN2_GPIO_Port     GPIOG
#define RL_IN2_Pin           GPIO_PIN_14

// 后右 RR
#define RR_IN1_GPIO_Port     GPIOE
#define RR_IN1_Pin           GPIO_PIN_1
#define RR_IN2_GPIO_Port     GPIOG
#define RR_IN2_Pin           GPIO_PIN_15


// 设置某个PWM通道占空比
static void Motor_SetPwm(MotorId_t id, uint16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;

    switch (id)
    {
        case MOTOR_FL:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            break;
        case MOTOR_FR:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
            break;
        case MOTOR_RL:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
            break;
        case MOTOR_RR:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
            break;
        default:
            break;
    }
}

// 设置方向引脚
static void Motor_SetDir(MotorId_t id, uint8_t forward)
{
    switch (id)
    {
        case MOTOR_FL:
            HAL_GPIO_WritePin(FL_IN1_GPIO_Port, FL_IN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(FL_IN2_GPIO_Port, FL_IN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;

        case MOTOR_FR:
            HAL_GPIO_WritePin(FR_IN1_GPIO_Port, FR_IN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(FR_IN2_GPIO_Port, FR_IN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;

        case MOTOR_RL:
            HAL_GPIO_WritePin(RL_IN1_GPIO_Port, RL_IN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RL_IN2_GPIO_Port, RL_IN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;

        case MOTOR_RR:
            HAL_GPIO_WritePin(RR_IN1_GPIO_Port, RR_IN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RR_IN2_GPIO_Port, RR_IN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;

        default:
            break;
    }
}

// 刹车/停止
static void Motor_Brake(MotorId_t id)
{
    switch (id)
    {
        case MOTOR_FL:
            HAL_GPIO_WritePin(FL_IN1_GPIO_Port, FL_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(FL_IN2_GPIO_Port, FL_IN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_FR:
            HAL_GPIO_WritePin(FR_IN1_GPIO_Port, FR_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(FR_IN2_GPIO_Port, FR_IN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_RL:
            HAL_GPIO_WritePin(RL_IN1_GPIO_Port, RL_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RL_IN2_GPIO_Port, RL_IN2_Pin, GPIO_PIN_RESET);
            break;

        case MOTOR_RR:
            HAL_GPIO_WritePin(RR_IN1_GPIO_Port, RR_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RR_IN2_GPIO_Port, RR_IN2_Pin, GPIO_PIN_RESET);
            break;

        default:
            break;
    }

    Motor_SetPwm(id, 0);
}

// 4轮归一化，防止超限
static void normalize_wheel_speed(int16_t *fl, int16_t *fr, int16_t *rl, int16_t *rr)
{
    int16_t max_val = ABS(*fl);

    if (ABS(*fr) > max_val) max_val = ABS(*fr);
    if (ABS(*rl) > max_val) max_val = ABS(*rl);
    if (ABS(*rr) > max_val) max_val = ABS(*rr);

    if (max_val > 1000)
    {
        *fl = (*fl) * 1000 / max_val;
        *fr = (*fr) * 1000 / max_val;
        *rl = (*rl) * 1000 / max_val;
        *rr = (*rr) * 1000 / max_val;
    }
}

// =========================

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    Motor_StopAll();
}

void Motor_SetSpeed(MotorId_t id, int16_t speed)
{
    uint16_t pwm;

    speed = limit_int16(speed, -1000, 1000);

    if (speed == 0)
    {
        Motor_Brake(id);
        return;
    }

    pwm = ABS(speed);

    if (speed > 0)
    {
        Motor_SetDir(id, 1);
    }
    else
    {
        Motor_SetDir(id, 0);
    }

    Motor_SetPwm(id, pwm);
}

void Motor_StopAll(void)
{
    Motor_Brake(MOTOR_FL);
    Motor_Brake(MOTOR_FR);
    Motor_Brake(MOTOR_RL);
    Motor_Brake(MOTOR_RR);
}

void Mecanum_SetSpeed(int16_t vx, int16_t vy, int16_t wz)
{
    int16_t fl, fr, rl, rr;

    //
    // 如果你发现平移方向反了，改 vy 正负
    // 如果你发现旋转方向反了，改 wz 正负
    // 如果单轮方向反了，改单个电机接线或SetDir逻辑
    // vx：前后速度
    // vy：左右平移速度
    // wz：旋转速度
    fl = vx - vy - wz;
    fr = vx + vy + wz;
    rl = vx + vy - wz;
    rr = vx - vy + wz;

    normalize_wheel_speed(&fl, &fr, &rl, &rr);

    Motor_SetSpeed(MOTOR_FL, fl);
    Motor_SetSpeed(MOTOR_FR, fr);
    Motor_SetSpeed(MOTOR_RL, rl);
    Motor_SetSpeed(MOTOR_RR, rr);
}
