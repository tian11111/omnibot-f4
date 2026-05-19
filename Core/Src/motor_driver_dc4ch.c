/**
 * @file  motor_driver_dc4ch.c
 * @brief 4-channel TB6612 motor driver via TIM8 PWM + dual-pin direction.
 *
 * TIM8 PWM pins (AF3, active high):
 *   CH1 -> PC6  (Front-Left  / "Ç°ÂÖA²à PWMA")
 *   CH2 -> PC7  (Front-Right / "Ç°ÂÖB²à PWMB")
 *   CH3 -> PC8  (Rear-Left   / "ºóÂÖA²à PWMA")  ? conflicts with stepper M2_DIR
 *   CH4 -> PC9  (Rear-Right  / "ºóÂÖB²à PWMB")  ? conflicts with stepper M2_DIR
 *
 * Direction pins (GPIO push-pull):
 *   Front-Left:  IN1=PB0  IN2=PB1
 *   Front-Right: IN1=PF13 IN2=PF14
 *   Rear-Left:   IN1=PE9  IN2=PE12
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
    /* [0] Front-Left  PWMA */
    { .htim = &htim8, .channel = TIM_CHANNEL_1,
      .in1_port = GPIOB, .in1_pin = GPIO_PIN_0,
      .in2_port = GPIOB, .in2_pin = GPIO_PIN_1,
      .invert = 0U },
    /* [1] Front-Right PWMB */
    { .htim = &htim8, .channel = TIM_CHANNEL_2,
      .in1_port = GPIOF, .in1_pin = GPIO_PIN_13,
      .in2_port = GPIOF, .in2_pin = GPIO_PIN_14,
      .invert = 0U },
    /* [2] Rear-Left   PWMA */
    { .htim = &htim8, .channel = TIM_CHANNEL_3,
      .in1_port = GPIOE, .in1_pin = GPIO_PIN_9,
      .in2_port = GPIOE, .in2_pin = GPIO_PIN_12,
      .invert = 0U },
    /* [3] Rear-Right  PWMB */
    { .htim = &htim8, .channel = TIM_CHANNEL_4,
      .in1_port = GPIOA, .in1_pin = GPIO_PIN_5,
      .in2_port = GPIOA, .in2_pin = GPIO_PIN_4,
      .invert = 0U },
};

/* ---- TIM8 handle (not in CubeMX tim.c, defined here) ---- */
TIM_HandleTypeDef htim8;

static void MX_TIM8_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM8_CLK_ENABLE();

    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 84U - 1U;   /* 84 MHz / 84 = 1 MHz tick */
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 100U - 1U;   /* 1 MHz / 100 = 10 kHz PWM */
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);
}

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
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* TIM8 PWM pins: PC6/PC7/PC8/PC9 -> AF3 */
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin       = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        gi.Mode      = GPIO_MODE_AF_PP;
        gi.Pull      = GPIO_NOPULL;
        gi.Speed     = GPIO_SPEED_FREQ_HIGH;
        gi.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &gi);
    }

    /* Direction pins */
    cfg_out(GPIOB, GPIO_PIN_0  | GPIO_PIN_1);   /* Front-Left  IN1/IN2 */
    cfg_out(GPIOF, GPIO_PIN_13 | GPIO_PIN_14);  /* Front-Right IN1/IN2 */
    cfg_out(GPIOE, GPIO_PIN_9  | GPIO_PIN_12);  /* Rear-Left   IN1/IN2 */
    cfg_out(GPIOA, GPIO_PIN_5  | GPIO_PIN_4);   /* Rear-Right  IN1/IN2 */

    /* Default: all coast (IN1=L, IN2=L) */
    for (uint8_t i = 0; i < DC4_MOTOR_COUNT; ++i)
    {
        tb6612_coast(&g_dc4_motors[i]);
    }

    /* Init TIM8 peripheral */
    MX_TIM8_PWM_Init();
}

void DC4_Motor_Start(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void DC4_Motor_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
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

