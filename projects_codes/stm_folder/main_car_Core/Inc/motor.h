/* motor.h */
#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"           // for TIM_HandleTypeDef and HAL macros
#include <stdint.h>

#define MOTOR_PWM_MAX          8399u

typedef struct {
    TIM_HandleTypeDef *pwm_timer;
    uint32_t           rpwm_channel;
    uint32_t           lpwm_channel;

    TIM_HandleTypeDef *encoder_timer;

    int32_t  current_encoder;
    int32_t  prev_encoder;
    int32_t  delta_encoder;
    float    rpm;

    float    counts_per_rev;            // encoder pulses per revolution (after ×4 quadrature)
} Motor_t;

/* ────────────────────────────────────────────────
   Public API
───────────────────────────────────────────────── */
void Motor_Init(
    Motor_t *motor,
    TIM_HandleTypeDef *pwm_tim,   uint32_t rpwm_ch, uint32_t lpwm_ch,
    TIM_HandleTypeDef *enc_tim,
    float counts_per_rev
);

void Motor_Start(Motor_t *motor);                    // starts PWM + encoder timer

void Motor_SetPWM(Motor_t *motor, int16_t pwm_signed);   // -MOTOR_PWM_MAX .. +MOTOR_PWM_MAX
void Motor_Stop(Motor_t *motor);

void Motor_UpdateEncoder(Motor_t *motor);
void Motor_ComputeRPM(Motor_t *motor, float dt_seconds);

#endif /* MOTOR_H_ */
