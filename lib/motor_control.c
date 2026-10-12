#include "motor_control.h"

// H?m kh?i t?o to?n b? c?u h?nh cho ??ng c?
void Motor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 1. C?p xung nh?p cho PORTA, PORTB, AFIO v? TIM4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 2. C?u h?nh GPIO
    // C?u h?nh c?c ch?n ?i?u h??ng (PA8, PA9, PA10, PA11) l? Output Push-Pull
    GPIO_InitStructure.GPIO_Pin = AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR_PORT, &GPIO_InitStructure);

    // C?u h?nh c?c ch?n PWM (PB6, PB7) l? Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Pin = PWMB_PIN | PWMA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_PORT, &GPIO_InitStructure);

    // 3. C?u h?nh Timer 4 ?? t?o PWM 20000Hz (20kHz - kh?ng nghe ti?ng r?t ??ng c?)
    /* 
     * Freq = 72,000,000 / ((Prescaler + 1) * (Period + 1))
     * Prescaler = 0 (h? s? chia 1), Period = MAX_SPEED - 1 (3599)
     * -> Freq = 72,000,000 / (1 * 3600) = 20,000 Hz = 20 kHz
     */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = MAX_SPEED - 1; // 3599
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // 4. C?u h?nh c?c k?nh PWM (Channel 1 v? 2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0; // Duty cycle ban ??u = 0%

    // K?nh 1 - PWMB (PB6) - B?nh ph?i
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // K?nh 2 - PWMA (PB7) - B?nh tr?i
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

// H?m ?i?u khi?n b?nh tr?i
// Quy ??c chu?n: S? d??ng (> 0) -> Ti?n, S? ?m (< 0) -> L?i, 0 -> D?ng
void MotorLeft_Control(int speed) {
    if (speed > 0) {
        // Tien (da dao chieu de phu hop phan cung)
        GPIO_ResetBits(DIR_PORT, AIN1_PIN);
        GPIO_SetBits(DIR_PORT, AIN2_PIN);
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        TIM_SetCompare2(TIM4, speed); 
    } else if (speed < 0) {
        // Lui (da dao chieu de phu hop phan cung)
        GPIO_SetBits(DIR_PORT, AIN1_PIN);
        GPIO_ResetBits(DIR_PORT, AIN2_PIN);
        if (speed < -MAX_SPEED) speed = -MAX_SPEED;
        TIM_SetCompare2(TIM4, -speed); 
    } else {
        // D?ng
        GPIO_ResetBits(DIR_PORT, AIN1_PIN | AIN2_PIN);
        TIM_SetCompare2(TIM4, 0);
    }
}

// H?m ?i?u khi?n b?nh ph?i
// Quy ??c chu?n: S? d??ng (> 0) -> Ti?n, S? ?m (< 0) -> L?i, 0 -> D?ng
void MotorRight_Control(int speed) {
    if (speed > 0) {
        // Ti?n
        GPIO_ResetBits(DIR_PORT, BIN1_PIN);
        GPIO_SetBits(DIR_PORT, BIN2_PIN);
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        TIM_SetCompare1(TIM4, speed); 
    } else if (speed < 0) {
        // L?i
        GPIO_SetBits(DIR_PORT, BIN1_PIN);
        GPIO_ResetBits(DIR_PORT, BIN2_PIN);
        if (speed < -MAX_SPEED) speed = -MAX_SPEED;
        TIM_SetCompare1(TIM4, -speed); 
    } else {
        // D?ng
        GPIO_ResetBits(DIR_PORT, BIN1_PIN | BIN2_PIN);
        TIM_SetCompare1(TIM4, 0);
    }
}
