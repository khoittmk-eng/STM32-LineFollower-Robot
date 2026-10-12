#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32f10x.h"

// ??nh ngh?a c?c ch?n PWM (Port B)
#define PWMB_PIN    GPIO_Pin_6 // B?nh ph?i - TIM4_CH1
#define PWMA_PIN    GPIO_Pin_7 // B?nh tr?i - TIM4_CH2
#define PWM_PORT    GPIOB

// ??nh ngh?a c?c ch?n ?i?u khi?n chi?u (Port A)
#define AIN2_PIN    GPIO_Pin_8
#define AIN1_PIN    GPIO_Pin_9
#define BIN1_PIN    GPIO_Pin_10
#define BIN2_PIN    GPIO_Pin_11
#define DIR_PORT    GPIOA

// Gi?i h?n t?c ?? l?n nh?t d?a tr?n Timer Period (T?n s? 20kHz -> Chu k? 3600)
#define MAX_SPEED   3600

// Khai b?o c?c h?m giao ti?p
void Motor_Init(void);
void MotorLeft_Control(int speed);
void MotorRight_Control(int speed);

#endif /* __MOTOR_CONTROL_H */
