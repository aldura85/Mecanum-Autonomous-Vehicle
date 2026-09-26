/* motor.c */
#include "motor.h"

void Motor_Init(
    Motor_t *motor,
    TIM_HandleTypeDef *pwm_tim, uint32_t rpwm_ch, uint32_t lpwm_ch,
    TIM_HandleTypeDef *enc_tim,
    float counts_per_rev
)
{
    motor->pwm_timer      = pwm_tim;
    motor->rpwm_channel   = rpwm_ch;
    motor->lpwm_channel   = lpwm_ch;
    motor->encoder_timer  = enc_tim;

    motor->current_encoder = 0;
    motor->prev_encoder    = 0;
    motor->delta_encoder   = 0;
    motor->rpm             = 0.0f;
    motor->counts_per_rev  = counts_per_rev;
}

void Motor_Start(Motor_t *motor)
{
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->rpwm_channel);
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->lpwm_channel);

    HAL_TIM_Encoder_Start(motor->encoder_timer, TIM_CHANNEL_ALL);
}

void Motor_SetPWM(Motor_t *motor, int16_t pwm_signed)
{
    uint32_t pwm_abs = (uint32_t)(pwm_signed < 0 ? -pwm_signed : pwm_signed);
    if (pwm_abs > MOTOR_PWM_MAX) pwm_abs = MOTOR_PWM_MAX;

    if (pwm_signed > 0)
    {
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->rpwm_channel, pwm_abs);
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->lpwm_channel, 0);
    }
    else if (pwm_signed < 0)
    {
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->rpwm_channel, 0);
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->lpwm_channel, pwm_abs);
    }
    else
    {
        Motor_Stop(motor);
    }
}

void Motor_Stop(Motor_t *motor)
{
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->rpwm_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->lpwm_channel, 0);
}

void Motor_UpdateEncoder(Motor_t *motor)
{
    int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(motor->encoder_timer);
    int32_t diff    = current - motor->prev_encoder;

    // Get the actual timer period (ARR value)
    uint32_t arr = motor->encoder_timer->Instance->ARR;

    // Calculate half the timer range
    int32_t half_range = (int32_t)(arr / 2u);

    // Correct for wrap-around in either direction
    if (diff > half_range)
        diff -= (int32_t)(arr + 1u);           // forward wrap
    else if (diff < -half_range)
        diff += (int32_t)(arr + 1u);           // backward wrap

    motor->delta_encoder   = diff;
    motor->current_encoder = current;
    motor->prev_encoder    = current;
}

void Motor_ComputeRPM(Motor_t *motor, float dt_sec)
{
    if (dt_sec < 1e-6f) return;

    // RPM = (delta counts / counts per rev) × (60 seconds / dt)
    motor->rpm = ((float)motor->delta_encoder / motor->counts_per_rev) * (60.0f / dt_sec);
}
