#ifndef MOTOR_DRIVER_DC4CH_H
#define MOTOR_DRIVER_DC4CH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define DC4_MOTOR_COUNT 4U

/* TB6612: dual-pin direction + PWM speed */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *in1_port;
    uint16_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint16_t           in2_pin;
    uint8_t            invert;   /* flip direction sense */
} DC4_MotorCfg;

void DC4_Motor_Init(void);
void DC4_Motor_Start(void);
void DC4_Motor_Stop(void);

/* signed_speed: -100..+100  (% duty) */
void DC4_Motor_SetSignedSpeed(uint8_t idx, int16_t signed_speed);
void DC4_Motor_AllStop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_DC4CH_H */

