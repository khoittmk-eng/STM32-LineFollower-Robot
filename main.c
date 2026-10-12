#include "stm32f10x.h"
#include "oled.h"
#include "motor_control.h"
#include "adc.h"
#include "LineSensor.h"
#include <stdio.h>

/* ==================================================================== */
/*                       CAU HINH NUT BAM (PORT B)                      */
/* ==================================================================== */
#define BTN_PORT            GPIOB
#define BTN_CLK             RCC_APB2Periph_GPIOB
#define BTN1_PIN            GPIO_Pin_0   // NEXT / UP
#define BTN2_PIN            GPIO_Pin_1   // SELECT / BACK (Giu 500ms de Back)
#define BTN3_PIN            GPIO_Pin_10  // START / STOP

/* ==================================================================== */
/*                       CAC TRANG THAI CUA MENU                        */
/* ==================================================================== */
typedef enum {
    MENU_MAIN = 0,
    MENU_RUN,
    MENU_SENSOR,
    MENU_PID,
    MENU_CALIB,
    MENU_MOTOR,
    MENU_SYSTEM
} MenuState_t;

MenuState_t currentMenu = MENU_MAIN;
uint8_t mainIndex = 0;       
uint8_t pidIndex = 0;        
uint8_t isEditingPID = 0;    
uint8_t isRunning = 0;       

/* ==================================================================== */
/*             TRANG THAI CHAY & TIM KIEM LINE CUA ROBOT                */
/* ==================================================================== */
typedef enum {
    ROBOT_RUN_NORMAL = 0,   // Dang chay bam line PID binh thuong
    ROBOT_RUN_PAUSE,        // Khoang nghi: Phanh dung xe de triet tieu quan tinh
    ROBOT_RUN_SEARCH,       // Xoay tai cho tim lai vach (Auto quay trai)
    ROBOT_RUN_LOST_STOP     // Tim qua timeout khong thay -> Dung han an toan
} RobotRunMode_t;

RobotRunMode_t robotRunMode = ROBOT_RUN_NORMAL;
uint16_t pauseCount = 0;
uint16_t searchCount = 0;
uint8_t  isBalanced = 0;      // 1: can bang tren line

#define PAUSE_CYCLES        30   // 30 chu ky * 5ms = 150ms nghi triet tieu quan tinh
#define SEARCH_SPEED        2000 // Toc do xoay tai cho tim vach (0 -> 3600)
#define SEARCH_TIMEOUT      600  // 600 chu ky * 5ms = 3.0 giay timeout

/* ==================================================================== */
/*                   BIEN CAM BIEN & THONG SO PID THUC TE               */
/* ==================================================================== */
uint16_t adc_raw[LINE_SENSOR_COUNT];
LineSensor_t line_sensor;

/* Thong so PID mac dinh */
float Kp = 17.00f, Ki = 0, Kd = 2.0f;
uint16_t baseSpeed = 2200;  // Thang 0 -> 3600 theo TIM4 20kHz
uint16_t maxSpeed  = 3600;  // 100% cong suat toi da cua Timer 4
#define CALIB_SPEED  3200   // Toc do xoay quet Auto Calib khoe hon de thang ma sat
float currentError = 0.0f;
float lastError    = 0.0f;
float integral     = 0.0f;

/* Bien trang thai dong co & vach line */
int current_pwmL = 0, current_pwmR = 0;
uint8_t lineDetected = 1;

/* Bien phu tro cho Menu Calib & Motor Test */
uint8_t calibState = 0;     // 0: Idle, 1: Auto Calib running, 2: Manual Calib running, 3: Done
uint8_t motorTestIdx = 0;   // 0: Stop, 1: Left Fwd, 2: Right Fwd, 3: Both Fwd, 4: Both Rev
uint8_t motorTestActive = 0;

/* ==================================================================== */
/*                       HAM TAO TRE VA KHOI TAO NUT                   */
/* ==================================================================== */
void Delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 7200; i++) {}
}

void Buttons_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(BTN_CLK, ENABLE);

    // Cau hinh Input Pull-Up (nhan nut se keo xuong 0V)
    GPIO_InitStruct.GPIO_Pin = BTN1_PIN | BTN2_PIN | BTN3_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(BTN_PORT, &GPIO_InitStruct);
}

/* ==================================================================== */
/*             HAM DIEU KHIEN DONG CO DONG BO HUONG CHAY                */
/* ==================================================================== */
/* Quy uoc chuan: > 0 la TIEN, < 0 la LUI, 0 la DUNG                    */
void Robot_SetSpeed(int left_speed, int right_speed) {
    MotorLeft_Control(left_speed);
    MotorRight_Control(right_speed);
}

void Robot_Stop(void) {
    MotorLeft_Control(0);
    MotorRight_Control(0);
    current_pwmL = 0;
    current_pwmR = 0;
}

/* ==================================================================== */
/*                 THUAT TOAN PID & DIEU KHIEN DO LINE                  */
/* ==================================================================== */
void PID_LineFollower_Update(void) {
    /* ---------------------------------------------------------------- */
    /* 1. TRANG THAI KHOANG NGHI (DUNG XE TRIET TIEU QUAN TINH ~150ms)  */
    /* ---------------------------------------------------------------- */
    if (robotRunMode == ROBOT_RUN_PAUSE) {
        Robot_Stop();
        pauseCount++;
        if (pauseCount >= PAUSE_CYCLES) {
            pauseCount = 0;
            searchCount = 0;
            integral = 0.0f;
            robotRunMode = ROBOT_RUN_SEARCH; // Het khoang nghi -> Bat dau xoay tim line
        }
        return;
    }

    /* ---------------------------------------------------------------- */
    /* 2. TRANG THAI XOAY TAI CHO TIM LAI LINE (AUTO QUAY TRAI)         */
    /* ---------------------------------------------------------------- */
    if (robotRunMode == ROBOT_RUN_SEARCH) {
        // Doc Digital xem mat nao da cham lai vao vach chua (line la bit 1)
        uint8_t dig = LineSensor_ReadDigital(&line_sensor, adc_raw);

        // Neu bat lai duoc line (co it nhat 1 mat la 1: dig != 0x00)
        if (dig != 0x00) {
            lineDetected = 1;
            robotRunMode = ROBOT_RUN_NORMAL;
            integral = 0.0f;
            // Chuyen ve bam line ngay trong chu ky nay
        } else {
            searchCount++;
            if (searchCount >= SEARCH_TIMEOUT) {
                // Tim qua timeout (3s) khong thay -> Dung han an toan
                Robot_Stop();
                robotRunMode = ROBOT_RUN_LOST_STOP;
                isRunning = 0;
                return;
            }

            // AUTO LUON XOAY SANG TRAI: Banh trai lui, Banh phai tien
            Robot_SetSpeed(-SEARCH_SPEED, SEARCH_SPEED);
            current_pwmL = -SEARCH_SPEED;
            current_pwmR = SEARCH_SPEED;
            return;
        }
    }

    /* ---------------------------------------------------------------- */
    /* 3. TRANG THAI DUNG HAN KHI TIM MAT LINE (TIMEOUT)                */
    /* ---------------------------------------------------------------- */
    if (robotRunMode == ROBOT_RUN_LOST_STOP) {
        Robot_Stop();
        return;
    }

    /* ---------------------------------------------------------------- */
    /* 4. TRANG THAI BAM LINE BINH THUONG (ROBOT_RUN_NORMAL)            */
    /* ---------------------------------------------------------------- */
    // 4.1. Lay sai so tu cam bien va doc Digital
    int32_t raw_error = LineSensor_GetError(&line_sensor, adc_raw);
    uint8_t dig = line_sensor.digital;

    // 4.2. Kiem tra Full 0: tat ca 8 mat deu la 0 -> VANG KHOI LINE
    if (dig == 0x00) {
        Robot_Stop(); // Phanh ngay lap tuc
        lineDetected = 0;
        isBalanced = 0;
        pauseCount = 0;
        robotRunMode = ROBOT_RUN_PAUSE; // Chuyen sang KHOANG NGHI truoc khi quay trai
        return;
    }

    // 4.3. Dang tren line binh thuong (co it nhat 1 mat la 1: dig != 0x00)
    lineDetected = 1;

    // Chuan hoa sai so ve khoang [-105.0, +105.0]
    currentError = (float)raw_error / 100.0f;

    // Tinh PID theo code ban dau
    float P = Kp * currentError;
    integral += currentError;

    // Gioi han tich phan chong tran (Anti-windup)
    if (integral > 1000.0f) integral = 1000.0f;
    if (integral < -1000.0f) integral = -1000.0f;
    float I = Ki * integral;

    float D = Kd * (currentError - lastError);
    lastError = currentError;

    float pid_out = P + I + D;

    // Tinh toc do 2 banh
    int left_speed  = (int)(baseSpeed + pid_out);
    int right_speed = (int)(baseSpeed - pid_out);

    // Kep toc do trong khoang gioi han maxSpeed
    if (left_speed > (int)maxSpeed)   left_speed = (int)maxSpeed;
    if (left_speed < -(int)maxSpeed)  left_speed = -(int)maxSpeed;
    if (right_speed > (int)maxSpeed)  right_speed = (int)maxSpeed;
    if (right_speed < -(int)maxSpeed) right_speed = -(int)maxSpeed;

    // Nhan dien TRANG THAI CAN BANG:
    // Toc do 2 banh chenh nhau <= 200 VA cac mat o giua bat duoc line (bit 1 o giua)
    int diff_speed = left_speed - right_speed;
    if (diff_speed < 0) diff_speed = -diff_speed;

    if (diff_speed <= 200 && ((dig & 0x18) != 0 || (dig & 0x3C) != 0)) {
        isBalanced = 1; // Can bang tren line
    } else {
        isBalanced = 0;
    }

    current_pwmL = left_speed;
    current_pwmR = right_speed;

    Robot_SetSpeed(left_speed, right_speed);
}

/* ==================================================================== */
/*                       HAM VE GIAO DIEN CAC MENU                      */
/* ==================================================================== */
void Draw_MainMenu(void) {
    const char* items[] = {
        "1. Run",
        "2. Sensor Monitor",
        "3. PID Setup",
        "4. Calibration",
        "5. Motor Test",
        "6. System"
    };
    
    OLED_DrawString(20, 2, "MAIN MENU", 1);
    OLED_DrawRect(0, 0, 128, 12, 1); 
    
    uint8_t startIdx = (mainIndex >= 4) ? (mainIndex - 3) : 0;
    
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t itemIdx = startIdx + i;
        if (itemIdx > 5) break;
        
        uint8_t y_pos = 16 + i * 12;
        if (itemIdx == mainIndex) {
            OLED_DrawString(2, y_pos, ">", 1); 
        }
        OLED_DrawString(12, y_pos, items[itemIdx], 1);
    }
}

void Draw_RunMenu(void) {
    char buf[32];
    OLED_DrawString(30, 0, "RUN MODE", 1);
    OLED_DrawRect(0, 0, 128, 10, 1);
    
    if (!isRunning) {
        OLED_DrawString(45, 12, "READY      ", 1);
    } else {
        switch (robotRunMode) {
            case ROBOT_RUN_NORMAL:
                if (isBalanced) OLED_DrawString(32, 12, "BALANCED   ", 1);
                else            OLED_DrawString(40, 12, "RUNNING    ", 1);
                break;
            case ROBOT_RUN_PAUSE:
                OLED_DrawString(35, 12, "PAUSE...   ", 1);
                break;
            case ROBOT_RUN_SEARCH:
                OLED_DrawString(25, 12, "SEARCH <- L", 1);
                break;
            case ROBOT_RUN_LOST_STOP:
                OLED_DrawString(25, 12, "LOST! STOP ", 1);
                break;
        }
    }
    
    snprintf(buf, sizeof(buf), "Error: %+.1f", currentError);
    OLED_DrawString(10, 25, buf, 1);
    
    snprintf(buf, sizeof(buf), "PWM L:%d R:%d", current_pwmL, current_pwmR); 
    OLED_DrawString(10, 38, buf, 1);
    
    // Hien thi Digital 8-bit: D0..D7 (Line = 1, Nen trang = 0 -> Full 0 la nen trang)
    uint8_t d = line_sensor.digital;
    snprintf(buf, sizeof(buf), "Dig: %d%d%d%d%d%d%d%d",
        (d & (1 << 0)) ? 1 : 0,
        (d & (1 << 1)) ? 1 : 0,
        (d & (1 << 2)) ? 1 : 0,
        (d & (1 << 3)) ? 1 : 0,
        (d & (1 << 4)) ? 1 : 0,
        (d & (1 << 5)) ? 1 : 0,
        (d & (1 << 6)) ? 1 : 0,
        (d & (1 << 7)) ? 1 : 0
    );
    OLED_DrawString(10, 52, buf, 1);
}

void Draw_SensorMenu(void) {
    char buf[32];
    OLED_DrawString(18, 0, "SENSOR MONITOR", 1);
    OLED_DrawRect(0, 0, 128, 10, 1);

    // Ve 8 cot tin hieu bieu thi 8 mat cam bien (Analog theo code goc)
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        uint8_t x = 8 + i * 14;
        OLED_DrawRect(x, 13, 10, 27, 1);

        uint16_t h = (line_sensor.sensorPID[i] * 25) / 3000;
        if (h > 25) h = 25;

        for (uint8_t bar_y = 0; bar_y < h; bar_y++) {
            for (uint8_t bar_x = 1; bar_x < 9; bar_x++) {
                SH1106_DrawPixel(x + bar_x, 39 - bar_y, 1);
            }
        }
    }

    // Hien thi Digital bitmask phia duoi cac cot (Line = 1, Nen trang = 0)
    uint8_t d = line_sensor.digital;
    snprintf(buf, sizeof(buf), "D:%d%d%d%d%d%d%d%d E:%+.1f",
        (d & (1 << 0)) ? 1 : 0,
        (d & (1 << 1)) ? 1 : 0,
        (d & (1 << 2)) ? 1 : 0,
        (d & (1 << 3)) ? 1 : 0,
        (d & (1 << 4)) ? 1 : 0,
        (d & (1 << 5)) ? 1 : 0,
        (d & (1 << 6)) ? 1 : 0,
        (d & (1 << 7)) ? 1 : 0,
        currentError
    );
    OLED_DrawString(0, 43, buf, 1);

    snprintf(buf, sizeof(buf), "Line:%d Dir:%s Bal:%d", 
        line_sensor.active_count,
        (line_sensor.last_dir > 0) ? "R" : ((line_sensor.last_dir < 0) ? "L" : "-"),
        isBalanced
    );
    OLED_DrawString(0, 54, buf, 1);
}

void Draw_PIDMenu(void) {
    char buf[32];
    OLED_DrawString(30, 0, "PID SETUP", 1);
    
    uint8_t startIdx = (pidIndex >= 3) ? (pidIndex - 2) : 0;
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t itemIdx = startIdx + i;
        if (itemIdx > 4) break;
        
        uint8_t y_pos = 15 + i * 15;
        
        if (itemIdx == pidIndex) {
            if (isEditingPID) OLED_DrawString(2, y_pos, "*", 1); 
            else              OLED_DrawString(2, y_pos, ">", 1); 
        }
        
        switch (itemIdx) {
            case 0: snprintf(buf, sizeof(buf), "Kp  : %.1f", Kp); break;
            case 1: snprintf(buf, sizeof(buf), "Ki  : %.2f", Ki); break;
            case 2: snprintf(buf, sizeof(buf), "Kd  : %.1f", Kd); break;
            case 3: snprintf(buf, sizeof(buf), "Base: %d", baseSpeed); break;
            case 4: snprintf(buf, sizeof(buf), "Max : %d", maxSpeed); break;
        }
        OLED_DrawString(15, y_pos, buf, 1);
    }
    
    OLED_DrawString(0, 56, "[Hold N2 to Back]", 1);
}

void Draw_CalibMenu(void) {
    OLED_DrawString(22, 0, "CALIBRATION", 1);
    OLED_DrawRect(0, 0, 128, 10, 1);

    if (calibState == 0) {
        OLED_DrawString(0, 15, "N1: Auto Calib (Spin)", 1);
        OLED_DrawString(0, 28, "N3: Manual Calib", 1);
        OLED_DrawString(0, 42, "Status: READY", 1);
    } else if (calibState == 1) {
        OLED_DrawString(10, 20, "AUTO CALIBRATING...", 1);
        OLED_DrawString(20, 36, "Spinning car...", 1);
    } else if (calibState == 2) {
        OLED_DrawString(10, 20, "MANUAL CALIB...", 1);
        OLED_DrawString(0, 35, "Sweep car over line", 1);
        OLED_DrawString(0, 48, "Press N3 to Finish", 1);
    } else if (calibState == 3) {
        OLED_DrawString(20, 25, "CALIB DONE!", 1);
        OLED_DrawString(0, 42, "Threshold calculated.", 1);
    }

    OLED_DrawString(0, 56, "[Hold N2 to Back]", 1);
}

void Draw_MotorMenu(void) {
    const char* motorItems[] = {
        "0. Stop All",
        "1. Left Fwd",
        "2. Right Fwd",
        "3. Both Fwd",
        "4. Both Rev"
    };

    OLED_DrawString(25, 0, "MOTOR TEST", 1);
    OLED_DrawRect(0, 0, 128, 10, 1);

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t y = 14 + i * 11;
        if (i == motorTestIdx) {
            OLED_DrawString(2, y, ">", 1);
        }
        OLED_DrawString(12, y, motorItems[i], 1);
    }

    if (motorTestActive) {
        OLED_DrawString(85, 45, "[RUN]", 1);
    } else {
        OLED_DrawString(85, 45, "[OFF]", 1);
    }

    OLED_DrawString(0, 56, "[Hold N2 to Back]", 1);
}

void Draw_SystemMenu(void) {
    OLED_DrawString(22, 0, "SYSTEM CONFIG", 1);
    OLED_DrawRect(0, 0, 128, 10, 1);

    OLED_DrawString(0, 14, "MCU: STM32F103C8T6", 1);
    OLED_DrawString(0, 26, "PWM: TIM4 - 20kHz", 1);
    OLED_DrawString(0, 38, "ADC: 8-Ch DMA Scan", 1);
    OLED_DrawString(0, 50, "N1: Reset PID Default", 1);
}

/* ==================================================================== */
/*                       CHUONG TRINH CHINH                             */
/* ==================================================================== */
int main(void) {
    // 1. Khoi tao cac ngoai vi
    SH1106_Init();
    Buttons_Init();
    ADC1_DMA_Init(adc_raw);
    Motor_Init();
    LineSensor_Init(&line_sensor);

    uint16_t btn1_cnt = 0, btn2_cnt = 0, btn3_cnt = 0;
    uint8_t ev_btn1_short = 0;
    uint8_t ev_btn2_short = 0, ev_btn2_long = 0;
    uint8_t ev_btn3_short = 0;

    uint32_t oled_refresh_cnt = 0;

    while (1) {
        /* ------------------------------------------------------------ */
        /* 1. XU LY NUT BAM                                             */
        /* ------------------------------------------------------------ */
        if (GPIO_ReadInputDataBit(BTN_PORT, BTN1_PIN) == 0) {
            if (btn1_cnt < 1000) btn1_cnt++;
        } else {
            if (btn1_cnt > 2) ev_btn1_short = 1; 
            btn1_cnt = 0;
        }

        if (GPIO_ReadInputDataBit(BTN_PORT, BTN2_PIN) == 0) {
            if (btn2_cnt < 1000) btn2_cnt++;
            if (btn2_cnt == 50) ev_btn2_long = 1; // Giu 500ms
        } else {
            if (btn2_cnt > 2 && btn2_cnt < 50) ev_btn2_short = 1; 
            btn2_cnt = 0;
        }

        if (GPIO_ReadInputDataBit(BTN_PORT, BTN3_PIN) == 0) {
            if (btn3_cnt < 1000) btn3_cnt++;
        } else {
            if (btn3_cnt > 2) ev_btn3_short = 1; 
            btn3_cnt = 0;
        }

        /* ------------------------------------------------------------ */
        /* 2. NUT 3 (START/STOP KHI O MENU RUN HOAC CALIB MANUAL)       */
        /* ------------------------------------------------------------ */
        if (ev_btn3_short) {
            ev_btn3_short = 0;
            if (currentMenu == MENU_RUN) {
                isRunning = !isRunning;
                if (!isRunning) {
                    Robot_Stop();
                    robotRunMode = ROBOT_RUN_NORMAL;
                    pauseCount = 0;
                    searchCount = 0;
                    integral = 0.0f;
                    isBalanced = 0;
                } else {
                    robotRunMode = ROBOT_RUN_NORMAL;
                    pauseCount = 0;
                    searchCount = 0;
                    integral = 0.0f;
                    lastError = 0.0f;
                    isBalanced = 0;
                }
            } else if (currentMenu == MENU_CALIB) {
                if (calibState == 0) {
                    calibState = 2;
                    LineSensor_StartCalib(&line_sensor);
                } else if (calibState == 2) {
                    LineSensor_FinishCalib(&line_sensor);
                    calibState = 3;
                }
            }
        }

        /* ------------------------------------------------------------ */
        /* 3. DIEU HUONG VA XU LY TRANG THAI MENU                       */
        /* ------------------------------------------------------------ */
        switch (currentMenu) {
            case MENU_MAIN:
                if (ev_btn1_short) { 
                    mainIndex = (mainIndex + 1) % 6; 
                }
                if (ev_btn2_short) {
                    if      (mainIndex == 0) currentMenu = MENU_RUN;
                    else if (mainIndex == 1) currentMenu = MENU_SENSOR;
                    else if (mainIndex == 2) currentMenu = MENU_PID;
                    else if (mainIndex == 3) {
                        currentMenu = MENU_CALIB;
                        calibState = 0;
                    }
                    else if (mainIndex == 4) {
                        currentMenu = MENU_MOTOR;
                        motorTestIdx = 0;
                        motorTestActive = 0;
                    }
                    else if (mainIndex == 5) currentMenu = MENU_SYSTEM;
                }
                break;

            case MENU_RUN:
                if (!isRunning) {
                    currentError = (float)LineSensor_GetError(&line_sensor, adc_raw) / 100.0f;
                }
                if (ev_btn2_long) {
                    isRunning = 0;
                    Robot_Stop();
                    robotRunMode = ROBOT_RUN_NORMAL;
                    currentMenu = MENU_MAIN;
                }
                break;

            case MENU_SENSOR:
                currentError = (float)LineSensor_GetError(&line_sensor, adc_raw) / 100.0f;
                if (ev_btn2_long) {
                    currentMenu = MENU_MAIN;
                }
                break;

            case MENU_PID:
                if (ev_btn1_short) {
                    if (!isEditingPID) {
                        pidIndex = (pidIndex + 1) % 5; 
                    } else {
                        if (pidIndex == 0) { Kp += 0.5f; if (Kp > 50.0f) Kp = 1.0f; }
                        if (pidIndex == 1) { Ki += 0.01f; if (Ki > 2.0f) Ki = 0.0f; }
                        if (pidIndex == 2) { Kd += 0.5f; if (Kd > 50.0f) Kd = 0.5f; }
                        if (pidIndex == 3) { baseSpeed += 100; if (baseSpeed > 3000) baseSpeed = 500; }
                        if (pidIndex == 4) { maxSpeed  += 100; if (maxSpeed > 3600)  maxSpeed = 1000; }
                    }
                }
                if (ev_btn2_short) {
                    isEditingPID = !isEditingPID;
                }
                if (ev_btn2_long) {
                    isEditingPID = 0;
                    currentMenu = MENU_MAIN;
                }
                break;

            case MENU_CALIB:
                // Nut 1: Kich hoat Auto Calib (Xe tu dong lac qua lai de quet vach)
                if (ev_btn1_short && calibState == 0) {
                    calibState = 1;
                    SH1106_Clear();
                    Draw_CalibMenu();
                    SH1106_UpdateScreen();

                    LineSensor_StartCalib(&line_sensor);

                    // Xoay nhe sang trai 1.5 giay voi toc do CALIB_SPEED (3200) de thang ma sat
                    Robot_SetSpeed(-CALIB_SPEED, CALIB_SPEED);
                    for (uint16_t t = 0; t < 150; t++) {
                        LineSensor_UpdateCalib(&line_sensor, adc_raw);
                        Delay_ms(10);
                    }
                    // Xoay nhe sang phai 3 giay de quet het vach
                    Robot_SetSpeed(CALIB_SPEED, -CALIB_SPEED);
                    for (uint16_t t = 0; t < 300; t++) {
                        LineSensor_UpdateCalib(&line_sensor, adc_raw);
                        Delay_ms(10);
                    }
                    // Xoay ve lai giua
                    Robot_SetSpeed(-CALIB_SPEED, CALIB_SPEED);
                    for (uint16_t t = 0; t < 150; t++) {
                        LineSensor_UpdateCalib(&line_sensor, adc_raw);
                        Delay_ms(10);
                    }

                    Robot_Stop();
                    LineSensor_FinishCalib(&line_sensor);
                    calibState = 3;
                }

                if (calibState == 2) {
                    LineSensor_UpdateCalib(&line_sensor, adc_raw);
                }

                if (ev_btn2_long) {
                    Robot_Stop();
                    calibState = 0;
                    currentMenu = MENU_MAIN;
                }
                break;

            case MENU_MOTOR:
                if (ev_btn1_short) {
                    motorTestIdx = (motorTestIdx + 1) % 5;
                    motorTestActive = 0;
                    Robot_Stop();
                }
                if (ev_btn2_short) {
                    motorTestActive = !motorTestActive;
                    if (motorTestActive) {
                        switch (motorTestIdx) {
                            case 0: Robot_Stop(); break;
                            case 1: Robot_SetSpeed(3600, 500); break;
                            case 2: Robot_SetSpeed(500, 3600); break;
                            case 3: Robot_SetSpeed(3600, 3600); break;
                            case 4: Robot_SetSpeed(-3000, -3000); break;
                        }
                    } else {
                        Robot_Stop();
                    }
                }
                if (ev_btn2_long) {
                    motorTestActive = 0;
                    Robot_Stop();
                    currentMenu = MENU_MAIN;
                }
                break;

            case MENU_SYSTEM:
                if (ev_btn1_short) {
                    Kp = 15.5f; Ki = 0.05f; Kd = 2.0f;
                    baseSpeed = 2800; maxSpeed = 3600;
                }
                if (ev_btn2_long) {
                    currentMenu = MENU_MAIN;
                }
                break;
        }

        ev_btn1_short = ev_btn2_short = ev_btn2_long = 0;

        /* ------------------------------------------------------------ */
        /* 4. CHAY THUAT TOAN DO LINE KHI O CHE DO RUN                  */
        /* ------------------------------------------------------------ */
        if (currentMenu == MENU_RUN && isRunning) {
            PID_LineFollower_Update();
        }

        /* ------------------------------------------------------------ */
        /* 5. CAP NHAT MAN HINH OLED (NON-BLOCKING TIMING)              */
        /* ------------------------------------------------------------ */
        oled_refresh_cnt++;
        if ((!isRunning) || (oled_refresh_cnt >= 20)) {
            oled_refresh_cnt = 0;
            SH1106_Clear();
            switch (currentMenu) {
                case MENU_MAIN:   Draw_MainMenu(); break;
                case MENU_RUN:    Draw_RunMenu(); break;
                case MENU_PID:    Draw_PIDMenu(); break;
                case MENU_SENSOR: Draw_SensorMenu(); break;
                case MENU_CALIB:  Draw_CalibMenu(); break;
                case MENU_MOTOR:  Draw_MotorMenu(); break;
                case MENU_SYSTEM: Draw_SystemMenu(); break;
            }
            SH1106_UpdateScreen();
        }

        Delay_ms(5);
    }
}
