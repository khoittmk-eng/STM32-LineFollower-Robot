#include "button.h"

static uint16_t btn1_cnt = 0;
static uint16_t btn2_cnt = 0;
static uint16_t btn3_cnt = 0;

void Buttons_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(BTN_CLK, ENABLE);

    // Cau hinh Input Pull-Up (nhan nut se keo xuong 0V)
    GPIO_InitStruct.GPIO_Pin = BTN1_PIN | BTN2_PIN | BTN3_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(BTN_PORT, &GPIO_InitStruct);
}

void Buttons_Scan(ButtonEvents_t *events) {
    events->btn1_short = 0;
    events->btn2_short = 0;
    events->btn2_long  = 0;
    events->btn3_short = 0;

    // Xu ly Nut 1 (PB0)
    if (GPIO_ReadInputDataBit(BTN_PORT, BTN1_PIN) == 0) {
        if (btn1_cnt < 1000) btn1_cnt++;
    } else {
        if (btn1_cnt > 2) events->btn1_short = 1; 
        btn1_cnt = 0;
    }

    // Xu ly Nut 2 (PB1 - Ho tro nhan giu 500ms)
    if (GPIO_ReadInputDataBit(BTN_PORT, BTN2_PIN) == 0) {
        if (btn2_cnt < 1000) btn2_cnt++;
        if (btn2_cnt == 50) events->btn2_long = 1; // Giu 50 chu ky * 10ms = 500ms
    } else {
        if (btn2_cnt > 2 && btn2_cnt < 50) events->btn2_short = 1; 
        btn2_cnt = 0;
    }

    // Xu ly Nut 3 (PB10)
    if (GPIO_ReadInputDataBit(BTN_PORT, BTN3_PIN) == 0) {
        if (btn3_cnt < 1000) btn3_cnt++;
    } else {
        if (btn3_cnt > 2) events->btn3_short = 1; 
        btn3_cnt = 0;
    }
}
