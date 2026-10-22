#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x.h"

/* ==================================================================== */
/*                       CAU HINH NUT BAM (PORT B)                      */
/* ==================================================================== */
#define BTN_PORT            GPIOB
#define BTN_CLK             RCC_APB2Periph_GPIOB
#define BTN1_PIN            GPIO_Pin_0   // NEXT / UP / TANG
#define BTN2_PIN            GPIO_Pin_1   // SELECT / BACK (Giu 500ms de Back)
#define BTN3_PIN            GPIO_Pin_10  // START / STOP / PREV / GIAM

typedef struct {
    uint8_t btn1_short;
    uint8_t btn2_short;
    uint8_t btn2_long;
    uint8_t btn3_short;
} ButtonEvents_t;

void Buttons_Init(void);
void Buttons_Scan(ButtonEvents_t *events);

#endif /* __BUTTON_H */
