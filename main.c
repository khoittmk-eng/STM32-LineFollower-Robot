#include "stm32f10x.h"
#include "oled.h"
#include "motor_control.h"
#include "adc.h"
#include "LineSensor.h"
#include "button.h"
#include "robot_control.h"
#include "menu.h"

/* ==================================================================== */
/*                       CHUONG TRINH CHINH                             */
/* ==================================================================== */
int main(void) {
    // 1. Khoi tao cac ngoai vi va module he thong
    SH1106_Init();
    Buttons_Init();
    ADC1_DMA_Init(adc_raw);
    Motor_Init();
    LineSensor_Init(&line_sensor);
    Robot_Init();
    Menu_Init();

    ButtonEvents_t btnEvents;
    uint32_t oled_refresh_cnt = 0;

    while (1) {
        /* ------------------------------------------------------------ */
        /* 1. QUET TRANG THAI NUT BAM                                   */
        /* ------------------------------------------------------------ */
        Buttons_Scan(&btnEvents);

        /* ------------------------------------------------------------ */
        /* 2. XU LY TRANG THAI MENU & DIEU HUONG                        */
        /* ------------------------------------------------------------ */
        Menu_Process(&btnEvents);

        /* ------------------------------------------------------------ */
        /* 3. CHAY THUAT TOAN DO LINE KHI O CHE DO RUN                  */
        /* ------------------------------------------------------------ */
        if (currentMenu == MENU_RUN && isRunning) 
					
				
				{
            Robot_PID_Update();
        }

        /* ------------------------------------------------------------ */
        /* 4. CAP NHAT MAN HINH OLED (NON-BLOCKING TIMING)              */
        /* ------------------------------------------------------------ */
        oled_refresh_cnt++;
        if ((!isRunning) || (oled_refresh_cnt >= 20)) {
            oled_refresh_cnt = 0;
            SH1106_Clear();
            Menu_Draw();
            SH1106_UpdateScreen();
        }

        Delay_ms(5);
    }
}
