#include "menu.h"
#include "oled.h"
#include "robot_control.h"
#include <stdio.h>

MenuState_t currentMenu = MENU_MAIN;
uint8_t mainIndex = 0;
uint8_t pidIndex = 0;
uint8_t isEditingPID = 0;
uint8_t calibState = 0;     // 0: Idle, 1: Auto Calib running, 2: Manual Calib running, 3: Done
uint8_t motorTestIdx = 0;   // 0: Stop, 1: Left Fwd, 2: Right Fwd, 3: Both Fwd, 4: Both Rev
uint8_t motorTestActive = 0;

void Menu_Init(void) {
    currentMenu = MENU_MAIN;
    mainIndex = 0;
    pidIndex = 0;
    isEditingPID = 0;
    calibState = 0;
    motorTestIdx = 0;
    motorTestActive = 0;
}

/* ==================================================================== */
/*                       HAM VE GIAO DIEN CAC MENU                      */
/* ==================================================================== */
static void Draw_MainMenu(void) {
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

static void Draw_RunMenu(void) {
    char buf[32];
    OLED_DrawString(30, 0, "RUN MODE", 1);
    OLED_DrawRect(0, 0, 128, 10, 1);
    
    if (!isRunning) {
        OLED_DrawString(45, 12, "READY      ", 1);
    } else {
        switch (robotRunMode) {
            case ROBOT_RUN_NORMAL:
                if (isBalanced) OLED_DrawString(32, 12, "MAX SPEED  ", 1);
                else            OLED_DrawString(32, 12, "TURN (BASE)", 1);
                break;
            case ROBOT_RUN_PAUSE:
                OLED_DrawString(35, 12, "PAUSE...   ", 1);
                break;
            case ROBOT_RUN_SEARCH:
                if (line_sensor.last_dir > 0) {
                    OLED_DrawString(25, 12, "SEARCH -> R", 1);
                } else {
                    OLED_DrawString(25, 12, "SEARCH <- L", 1);
                }
                break;
            case ROBOT_RUN_LOST_STOP:
                OLED_DrawString(25, 12, "LOST! STOP ", 1);
                break;
        }
    }
    
    snprintf(buf, sizeof(buf), "Error: %+.1f", currentError);
    OLED_DrawString(10, 25, buf, 1);
    
    // Hien thi toc do tren thang 100 (%)
    snprintf(buf, sizeof(buf), "PWM L:%d%% R:%d%%", current_pwmL, current_pwmR); 
    OLED_DrawString(10, 38, buf, 1);
    
    // Hien thi Digital 8-bit: D0..D7 (Line = 1, Nen trang = 0)
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

static void Draw_SensorMenu(void) {
    char buf[32];
    OLED_DrawString(18, 0, "SENSOR MONITOR", 1);
    OLED_DrawRect(0, 0, 128, 10, 1);

    // Ve 8 cot tin hieu bieu thi 8 mat cam bien Analog
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

    // Hien thi Digital bitmask phia duoi cac cot
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

static void Draw_PIDMenu(void) {
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
            case 3: snprintf(buf, sizeof(buf), "Base: %d%%", baseSpeed); break;
            case 4: snprintf(buf, sizeof(buf), "Max : %d%%", maxSpeed); break;
        }
        OLED_DrawString(15, y_pos, buf, 1);
    }
    
    // Huong dan nut bam
    if (isEditingPID) {
        OLED_DrawString(0, 56, "N1:+  N3:-  N2:OK", 1);
    } else {
        OLED_DrawString(0, 56, "N2:Edit [Hold:Back]", 1);
    }
}

static void Draw_CalibMenu(void) {
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

static void Draw_MotorMenu(void) {
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

static void Draw_SystemMenu(void) {
    OLED_DrawString(22, 0, "SYSTEM CONFIG", 1);
    OLED_DrawRect(0, 0, 128, 10, 1);

    OLED_DrawString(0, 14, "MCU: STM32F103C8T6", 1);
    OLED_DrawString(0, 26, "PWM: TIM4 - 20kHz", 1);
    OLED_DrawString(0, 38, "ADC: 8-Ch DMA Scan", 1);
    OLED_DrawString(0, 50, "N1: Reset PID Default", 1);
}

void Menu_Draw(void) {
    switch (currentMenu) {
        case MENU_MAIN:   Draw_MainMenu();   break;
        case MENU_RUN:    Draw_RunMenu();    break;
        case MENU_SENSOR: Draw_SensorMenu(); break;
        case MENU_PID:    Draw_PIDMenu();    break;
        case MENU_CALIB:  Draw_CalibMenu();  break;
        case MENU_MOTOR:  Draw_MotorMenu();  break;
        case MENU_SYSTEM: Draw_SystemMenu(); break;
    }
}

/* ==================================================================== */
/*                       XU LY DIEU HUONG VA NUT BAM                    */
/* ==================================================================== */
void Menu_Process(ButtonEvents_t *events) {
    // 1. Nut 3 rieng cho che do RUN va CALIB MANUAL
    if (events->btn3_short) {
        if (currentMenu == MENU_RUN) {
            events->btn3_short = 0;
            Robot_ToggleRun();
        } else if (currentMenu == MENU_CALIB) {
            events->btn3_short = 0;
            if (calibState == 0) {
                calibState = 2;
                LineSensor_StartCalib(&line_sensor);
            } else if (calibState == 2) {
                LineSensor_FinishCalib(&line_sensor);
                calibState = 3;
            }
        }
    }

    // 2. Xu ly theo tung man hinh cu the
    switch (currentMenu) {
        case MENU_MAIN:
            if (events->btn1_short) {
                mainIndex = (mainIndex + 1) % 6;
            }
            if (events->btn3_short) {
                mainIndex = (mainIndex == 0) ? 5 : (mainIndex - 1);
            }
            if (events->btn2_short) {
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
            if (events->btn2_long) {
                isRunning = 0;
                Robot_Stop();
                robotRunMode = ROBOT_RUN_NORMAL;
                currentMenu = MENU_MAIN;
            }
            break;

        case MENU_SENSOR:
            currentError = (float)LineSensor_GetError(&line_sensor, adc_raw) / 100.0f;
            if (events->btn2_long) {
                currentMenu = MENU_MAIN;
            }
            break;

        case MENU_PID:
            // Nut 1: Neu dang chon -> di chuyen xuong. Neu dang sua -> TANG gia tri
            if (events->btn1_short) {
                if (!isEditingPID) {
                    pidIndex = (pidIndex + 1) % 5;
                } else {
                    switch (pidIndex) {
                        case 0: Kp += 0.5f; if (Kp > 100.0f) Kp = 100.0f; break;
                        case 1: Ki += 0.01f; if (Ki > 5.0f) Ki = 5.0f; break;
                        case 2: Kd += 0.5f; if (Kd > 100.0f) Kd = 100.0f; break;
                        case 3: baseSpeed += 5; if (baseSpeed > 100) baseSpeed = 100; break;
                        case 4: maxSpeed  += 5; if (maxSpeed > 100)  maxSpeed = 100; break;
                    }
                }
            }

            // Nut 3: Neu dang chon -> di chuyen len. Neu dang sua -> GIAM gia tri (Theo yeu cau)
            if (events->btn3_short) {
                if (!isEditingPID) {
                    pidIndex = (pidIndex == 0) ? 4 : (pidIndex - 1);
                } else {
                    switch (pidIndex) {
                        case 0: if (Kp >= 0.5f) Kp -= 0.5f; else Kp = 0.0f; break;
                        case 1: if (Ki >= 0.01f) Ki -= 0.01f; else Ki = 0.0f; break;
                        case 2: if (Kd >= 0.5f) Kd -= 0.5f; else Kd = 0.0f; break;
                        case 3: if (baseSpeed >= 5) baseSpeed -= 5; else baseSpeed = 0; break;
                        case 4: if (maxSpeed >= 5) maxSpeed -= 5; else maxSpeed = 0; break;
                    }
                }
            }

            // Nut 2 bam ngan: Chuyen doi trang thai Chinh sua / Lua chon
            if (events->btn2_short) {
                isEditingPID = !isEditingPID;
            }

            // Nut 2 nhan giu: Quay ve Main Menu
            if (events->btn2_long) {
                isEditingPID = 0;
                currentMenu = MENU_MAIN;
            }
            break;

        case MENU_CALIB:
            // Nut 1: Kich hoat Auto Calib (Xe tu dong lac qua lai de quet vach voi CALIB_SPEED)
            if (events->btn1_short && calibState == 0) {
                calibState = 1;
                SH1106_Clear();
                Draw_CalibMenu();
                SH1106_UpdateScreen();

                LineSensor_StartCalib(&line_sensor);

                // Xoay trai 1.5s voi toc do CALIB_SPEED (thang 100)
                Robot_SetSpeed(-CALIB_SPEED, CALIB_SPEED);
                for (uint16_t t = 0; t < 150; t++) {
                    LineSensor_UpdateCalib(&line_sensor, adc_raw);
                    Delay_ms(10);
                }
                // Xoay phai 3.0s de quet het vach
                Robot_SetSpeed(CALIB_SPEED, -CALIB_SPEED);
                for (uint16_t t = 0; t < 300; t++) {
                    LineSensor_UpdateCalib(&line_sensor, adc_raw);
                    Delay_ms(10);
                }
                // Xoay ve lai giua 1.5s
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

            if (events->btn2_long) {
                Robot_Stop();
                calibState = 0;
                currentMenu = MENU_MAIN;
            }
            break;

        case MENU_MOTOR:
            if (events->btn1_short) {
                motorTestIdx = (motorTestIdx + 1) % 5;
                motorTestActive = 0;
                Robot_Stop();
            }
            if (events->btn3_short) {
                motorTestIdx = (motorTestIdx == 0) ? 4 : (motorTestIdx - 1);
                motorTestActive = 0;
                Robot_Stop();
            }
            if (events->btn2_short) {
                motorTestActive = !motorTestActive;
                if (motorTestActive) {
                    switch (motorTestIdx) {
                        case 0: Robot_Stop(); break;
                        case 1: Robot_SetSpeed(100, 15); break;
                        case 2: Robot_SetSpeed(15, 100); break;
                        case 3: Robot_SetSpeed(100, 100); break;
                        case 4: Robot_SetSpeed(-80, -80); break;
                    }
                } else {
                    Robot_Stop();
                }
            }
            if (events->btn2_long) {
                motorTestActive = 0;
                Robot_Stop();
                currentMenu = MENU_MAIN;
            }
            break;

        case MENU_SYSTEM:
            if (events->btn1_short) {
                Robot_ResetPID_Default();
            }
            if (events->btn2_long) {
                currentMenu = MENU_MAIN;
            }
            break;
    }
}
