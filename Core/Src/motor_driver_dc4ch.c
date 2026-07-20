/**
 * @file  motor_driver_dc4ch.c
 * @brief 4-channel TB6612 motor driver via TIM1/TIM5 PWM + dual-pin direction.
 *
 * PWM pins:
 *   PA2 (TIM5_CH3) -> Front-Left  PWMA
 *   PA3 (TIM5_CH4) -> Front-Right PWMB
 *   PE11 (TIM1_CH2) -> Rear-Left   PWMA
 *   PE13 (TIM1_CH3) -> Rear-Right  PWMB
 *
 * Direction pins (GPIO push-pull):
 *   Front-Left:  IN1=PB0  IN2=PB1
 *   Front-Right: IN1=PF13 IN2=PF14
 *   Rear-Left:   IN1=PE12 IN2=PE9
 *   Rear-Right:  IN1=PA5  IN2=PA4
 *
 * TB6612 truth table:
 *   IN1=H  IN2=L  PWM>0  -> Forward
 *   IN1=L  IN2=H  PWM>0  -> Reverse
 *   IN1=H  IN2=H  any    -> Brake
 *   IN1=L  IN2=L  any    -> Coast (stop)
 */

#include "motor_driver_dc4ch.h"
#include "tim.h"
#include <stdlib.h>

/* ---- Motor configuration table ---- */
static const DC4_MotorCfg g_dc4_motors[DC4_MOTOR_COUNT] = {
    /* [0] Front-Left  PWMA - PA2 (TIM5_CH3) */
    /* invert=1: FL wiring polarity corrected */
    { .htim = &htim5, .channel = TIM_CHANNEL_3,
      .in1_port = GPIOB, .in1_pin = GPIO_PIN_0,
      .in2_port = GPIOB, .in2_pin = GPIO_PIN_1,
      .invert = 1U },
    /* [1] Front-Right PWMB - PA3 (TIM5_CH4) */
    { .htim = &htim5, .channel = TIM_CHANNEL_4,
      .in1_port = GPIOF, .in1_pin = GPIO_PIN_13,
      .in2_port = GPIOF, .in2_pin = GPIO_PIN_14,
      .invert = 0U },
    /* [2] Rear-Left   PWMA - PE11 (TIM1_CH2), IN1=PE12, IN2=PE9 */
    /* invert=1: RL wiring polarity corrected */
    { .htim = &htim1, .channel = TIM_CHANNEL_2,
      .in1_port = GPIOE, .in1_pin = GPIO_PIN_12,
      .in2_port = GPIOE, .in2_pin = GPIO_PIN_9,
      .invert = 1U },
    /* [3] Rear-Right  PWMB - PE13 (TIM1_CH3) */
    { .htim = &htim1, .channel = TIM_CHANNEL_3,
      .in1_port = GPIOA, .in1_pin = GPIO_PIN_5,
      .in2_port = GPIOA, .in2_pin = GPIO_PIN_4,
      .invert = 1U },
};

/* ---- GPIO helpers ---- */
static void cfg_out(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gi);
}

/* ---- TB6612 direction control ---- */
static void tb6612_forward(const DC4_MotorCfg *m)
{
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
}

static void tb6612_reverse(const DC4_MotorCfg *m)
{
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
}

static void tb6612_brake(const DC4_MotorCfg *m)
{
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
}

static void tb6612_coast(const DC4_MotorCfg *m)
{
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
}

/* ---- Public API ---- */

void DC4_Motor_Init(void)
{
    /* Enable GPIO clocks for direction pins */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Disable TIM1_CH1 on PE9 - we need PE9 as GPIO for direction control */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

    /* Direction pins */
    cfg_out(GPIOB, GPIO_PIN_0  | GPIO_PIN_1);   /* Front-Left  IN1/IN2 */
    cfg_out(GPIOF, GPIO_PIN_13 | GPIO_PIN_14);  /* Front-Right IN1/IN2 */
    cfg_out(GPIOE, GPIO_PIN_12 | GPIO_PIN_9);   /* Rear-Left   IN1=PE12, IN2=PE9 */
    cfg_out(GPIOA, GPIO_PIN_5  | GPIO_PIN_4);   /* Rear-Right  IN1/IN2 */

    /* Default: all coast (IN1=L, IN2=L) */
    for (uint8_t i = 0; i < DC4_MOTOR_COUNT; ++i)
    {
        tb6612_coast(&g_dc4_motors[i]);
    }

    /* TIM1 PWM is initialized by CubeMX (MX_TIM1_Init) - no manual init needed */
}

void DC4_Motor_Start(void)
{
    /* Front wheels use TIM5 CH3/CH4 (PA2/PA3) */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

    /* Rear wheels use TIM1 CH2/CH3 (PE11/PE13) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void DC4_Motor_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
}

void DC4_Motor_SetSignedSpeed(uint8_t idx, int16_t signed_speed)
{
    if (idx >= DC4_MOTOR_COUNT) return;
    if (signed_speed >  100) signed_speed =  100;
    if (signed_speed < -100) signed_speed = -100;

    const DC4_MotorCfg *m = &g_dc4_motors[idx];

    /* Apply invert flag */
    int16_t spd = m->invert ? -signed_speed : signed_speed;

    /* Direction */
    if (spd > 0)      tb6612_forward(m);
    else if (spd < 0)  tb6612_reverse(m);
    else               tb6612_brake(m);   /* 0% = brake */

    /* PWM duty: |spd| % of ARR */
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(m->htim) + 1U;
    const uint32_t ccr = (uint32_t)((uint32_t)abs(spd) * arr / 100U);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, ccr);
}

void DC4_Motor_AllStop(void)
{
    for (uint8_t i = 0U; i < DC4_MOTOR_COUNT; ++i)
    {
        DC4_Motor_SetSignedSpeed(i, 0);
    }
}

