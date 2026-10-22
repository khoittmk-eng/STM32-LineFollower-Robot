#include "robot_control.h"
#include "motor_control.h"
#include "adc.h"

/* Bien cam bien */
uint16_t adc_raw[LINE_SENSOR_COUNT];
LineSensor_t line_sensor;

/* Trang thai robot */
RobotRunMode_t robotRunMode = ROBOT_RUN_NORMAL;
uint8_t isRunning = 0;
uint8_t isBalanced = 0;
uint8_t lineDetected = 1;

static uint16_t pauseCount = 0;
static uint16_t searchCount = 0;
static uint16_t straightCount = 0;

/* Thong so PID mac dinh */
float Kp = 20.0f;
float Ki = 0.0f;
float Kd = 25.0f;

/* Hieu chuan toc do ve thang 100 (%) */
uint16_t baseSpeed = 60;   // 60% cong suat (~2160 PWM tren Timer 4)
uint16_t maxSpeed  = 100;  // 100% cong suat toi da

float currentError = 0.0f;
float lastError    = 0.0f;
float integral     = 0.0f;

/* Toc do thuc te tren thang 100 (-100% den +100%) dung de hien thi */
int current_pwmL = 0;
int current_pwmR = 0;

void Delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 7200; i++) {}
}

void Robot_Init(void) {
    robotRunMode = ROBOT_RUN_NORMAL;
    isRunning = 0;
    isBalanced = 0;
    lineDetected = 1;
    pauseCount = 0;
    searchCount = 0;
    straightCount = 0;
    currentError = 0.0f;
    lastError = 0.0f;
    integral = 0.0f;
    current_pwmL = 0;
    current_pwmR = 0;
    line_sensor.last_dir = 0;
}

/* ==================================================================== */
/*             HAM DIEU KHIEN DONG CO VOI THANG DO 100 (-100 -> +100)   */
/* ==================================================================== */
void Robot_SetSpeed(int left_percent, int right_percent) {
    if (left_percent > 100) left_percent = 100;
    if (left_percent < -100) left_percent = -100;
    if (right_percent > 100) right_percent = 100;
    if (right_percent < -100) right_percent = -100;

    // Quy doi tu thang 100 sang chu ky Timer 4 (MAX_SPEED = 3600)
    int left_pwm  = (left_percent  * MAX_SPEED) / 100;
    int right_pwm = (right_percent * MAX_SPEED) / 100;

    MotorLeft_Control(left_pwm);
    MotorRight_Control(right_pwm);

    current_pwmL = left_percent;
    current_pwmR = right_percent;
}

void Robot_Stop(void) {
    MotorLeft_Control(0);
    MotorRight_Control(0);
    current_pwmL = 0;
    current_pwmR = 0;
}

void Robot_ToggleRun(void) {
    isRunning = !isRunning;
    if (!isRunning) {
        Robot_Stop();
        robotRunMode = ROBOT_RUN_NORMAL;
        pauseCount = 0;
        searchCount = 0;
        straightCount = 0;
        integral = 0.0f;
        isBalanced = 0;
    } else {
        robotRunMode = ROBOT_RUN_NORMAL;
        pauseCount = 0;
        searchCount = 0;
        straightCount = 0;
        integral = 0.0f;
        lastError = 0.0f;
        isBalanced = 0;
        line_sensor.last_dir = 0;
    }
}

void Robot_ResetPID_Default(void) {
    Kp = 15.5f;
    Ki = 0.05f;
    Kd = 2.0f;
    baseSpeed = 75; // 75%
    maxSpeed  = 100; // 100%
}

/* ==================================================================== */
/*                 THUAT TOAN PID & DIEU KHIEN DO LINE                  */
/* ==================================================================== */
void Robot_PID_Update(void) {
    /* ---------------------------------------------------------------- */
    /* 1. TRANG THAI KHOANG NGHI (DUNG XE TRIET TIEU QUAN TINH ~75ms)   */
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
    /* 2. TRANG THAI XOAY TAI CHO TIM LAI LINE                          */
    /* ---------------------------------------------------------------- */
    if (robotRunMode == ROBOT_RUN_SEARCH) {
        // Doc Digital xem mat nao da cham lai vao vach chua (line la bit 1)
        uint8_t dig = LineSensor_ReadDigital(&line_sensor, adc_raw);

        // Neu bat lai duoc line (co it nhat 1 mat la 1: dig != 0x00)
        if (dig != 0x00) {
            lineDetected = 1;
            robotRunMode = ROBOT_RUN_NORMAL;
            integral = 0.0f;
        } else {
            searchCount++;
            if (searchCount >= SEARCH_TIMEOUT) {
                // Tim qua timeout (3s) khong thay -> Dung han an toan
                Robot_Stop();
                robotRunMode = ROBOT_RUN_LOST_STOP;
                isRunning = 0;
                return;
            }

            // Xoay ve ben ma lan gan nhat bat duoc line co nhieu mat hon:
            // last_dir > 0: ben phai nhieu hon -> Xoay sang phai (Banh trai tien, banh phai lui)
            // last_dir <= 0: ben trai nhieu hon (hoac mac dinh) -> Xoay sang trai (Banh trai lui, banh phai tien)
            if (line_sensor.last_dir > 0) {
                Robot_SetSpeed(SEARCH_SPEED, -SEARCH_SPEED);
            } else {
                Robot_SetSpeed(-SEARCH_SPEED, SEARCH_SPEED);
            }
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
    int32_t raw_error = LineSensor_GetError(&line_sensor, adc_raw);
    uint8_t dig = line_sensor.digital;

    // Kiem tra Full 0: tat ca 8 mat deu la 0 -> VANG KHOI LINE
    if (dig == 0x00) {
        Robot_Stop(); // Phanh ngay lap tuc
        lineDetected = 0;
        isBalanced = 0;
        straightCount = 0;
        pauseCount = 0;
        robotRunMode = ROBOT_RUN_PAUSE; // Chuyen sang KHOANG NGHI truoc khi quay trai
        return;
    }

    // Dang tren line binh thuong (co it nhat 1 mat la 1: dig != 0x00)
    lineDetected = 1;

    // Chuan hoa sai so ve khoang [-105.0, +105.0]
    currentError = (float)raw_error / 100.0f;

    // Tinh toan PID
    float P = Kp * currentError;
    integral += currentError;

    // Gioi han tich phan chong tran (Anti-windup)
    if (integral > 1000.0f) integral = 1000.0f;
    if (integral < -1000.0f) integral = -1000.0f;
    float I = Ki * integral;

    float D = Kd * (currentError - lastError);
    lastError = currentError;

    float pid_out = P + I + D;

    // 1. Chuyen baseSpeed va maxSpeed tu thang 100 sang PWM thuc te (MAX_SPEED = 3600)
    int base_pwm = (baseSpeed * MAX_SPEED) / 100;
    int max_pwm  = (maxSpeed  * MAX_SPEED) / 100;

    // 2. Nhan dien duong thang vs luc re:
    // - Co mat o giua bat duoc line: mat 3 va/hoac mat 4 (dig & 0x18 != 0)
    // - Khong co mat bien ngoai cung nao bat line: mat 0, 1 (trai) va mat 6, 7 (phai) deu bang 0 (dig & 0xC3 == 0)
    // - Sai so currentError nho trong vung tam: |currentError| <= 22.0f
    float absError = (currentError < 0.0f) ? -currentError : currentError;
    if (((dig & 0x18) != 0) && ((dig & 0xC3) == 0) && (absError <= 22.0f)) {
        if (straightCount < 6) {
            straightCount++;
        }
        if (straightCount >= 6) {
            isBalanced = 1; // Xac nhan dang o duong thang on dinh -> Day max toc do
        }
    } else {
        straightCount = 0;
        isBalanced = 0;     // Luc re -> Ha ve baseSpeed +- PID de vao cua an toan
    }

    // 3. Tinh toan toc do 2 banh theo PWM:
    // - Duong thang: Day max toc do
    // - Luc re: Dua van toc 2 banh ve baseSpeed + - PID
    int left_pwm;
    int right_pwm;

    if (isBalanced) {
        // DUONG THANG: Day max toc
        int straight_base = max_pwm;
        left_pwm  = straight_base + (int)pid_out;
        right_pwm = straight_base - (int)pid_out;

        // Phan phoi headroom: neu 1 banh vuot max_pwm thi kep o max_pwm
        // va tru bot o banh doi dien de bao toan day du do lech PID (2 * pid_out)
        if (left_pwm > max_pwm) {
            int excess = left_pwm - max_pwm;
            left_pwm = max_pwm;
            right_pwm -= excess;
        }
        if (right_pwm > max_pwm) {
            int excess = right_pwm - max_pwm;
            right_pwm = max_pwm;
            left_pwm -= excess;
        }
    } else {
        // LUC RE: Dua ve baseSpeed + - PID
        left_pwm  = base_pwm + (int)pid_out;
        right_pwm = base_pwm - (int)pid_out;
    }

    // Kep toc do trong khoang gioi han an toan [-max_pwm, max_pwm]
    if (left_pwm > max_pwm)   left_pwm = max_pwm;
    if (left_pwm < -max_pwm)  left_pwm = -max_pwm;
    if (right_pwm > max_pwm)  right_pwm = max_pwm;
    if (right_pwm < -max_pwm) right_pwm = -max_pwm;

    // Xuat tin hieu dieu khien dong co
    MotorLeft_Control(left_pwm);
    MotorRight_Control(right_pwm);

    // Quy doi ve thang 100 (%) cho hien thi OLED
    current_pwmL = (left_pwm  * 100) / MAX_SPEED;
    current_pwmR = (right_pwm * 100) / MAX_SPEED;
}
