#ifndef __ROBOT_CONTROL_H
#define __ROBOT_CONTROL_H

#include "stm32f10x.h"
#include "LineSensor.h"

/* ==================================================================== */
/*             TRANG THAI CHAY & TIM KIEM LINE CUA ROBOT                */
/* ==================================================================== */
typedef enum {
    ROBOT_RUN_NORMAL = 0,   // Dang chay bam line PID binh thuong
    ROBOT_RUN_PAUSE,        // Khoang nghi: Phanh dung xe de triet tieu quan tinh
    ROBOT_RUN_SEARCH,       // Xoay tai cho tim lai vach theo huong lech lan gan nhat
    ROBOT_RUN_LOST_STOP     // Tim qua timeout khong thay -> Dung han an toan
} RobotRunMode_t;

/* ==================================================================== */
/*       HANG SO DINH NGHIA TOC DO & THOI GIAN (THANG 100)              */
/* ==================================================================== */
#define PAUSE_CYCLES        30  // 15 chu ky * 5ms = 75ms nghi triet tieu quan tinh
#define SEARCH_SPEED        70   // Thang 100: Toc do xoay tai cho tim vach (70%)
#define SEARCH_TIMEOUT      600  // 600 chu ky * 5ms = 3.0 giay timeout
#define CALIB_SPEED         90   // Thang 100: Toc do xoay Auto Calib thang ma sat (90%)

/* ==================================================================== */
/*                   BIEN CAM BIEN & THONG SO ROBOT                     */
/* ==================================================================== */
extern RobotRunMode_t robotRunMode;
extern uint8_t isRunning;
extern uint8_t isBalanced;
extern uint8_t lineDetected;

/* Thong so PID & Toc do (Hieu chuan theo thang 100) */
extern float Kp;
extern float Ki;
extern float Kd;
extern uint16_t baseSpeed;  // Thang 0 -> 100 (%) (Mac dinh: 60)
extern uint16_t maxSpeed;   // Thang 0 -> 100 (%) (Mac dinh: 100)

extern float currentError;
extern float lastError;
extern float integral;

/* Toc do hien tai tren thang 100 (-100 den +100%) de hien thi */
extern int current_pwmL;
extern int current_pwmR;

/* Cam bien */
extern uint16_t adc_raw[LINE_SENSOR_COUNT];
extern LineSensor_t line_sensor;

/* ==================================================================== */
/*                       CAC HAM DIEU KHIEN                             */
/* ==================================================================== */
void Delay_ms(uint32_t ms);
void Robot_Init(void);
void Robot_SetSpeed(int left_percent, int right_percent);
void Robot_Stop(void);
void Robot_PID_Update(void);
void Robot_ToggleRun(void);
void Robot_ResetPID_Default(void);

#endif /* __ROBOT_CONTROL_H */
