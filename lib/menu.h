#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"
#include "button.h"

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

extern MenuState_t currentMenu;
extern uint8_t mainIndex;
extern uint8_t pidIndex;
extern uint8_t isEditingPID;
extern uint8_t calibState;
extern uint8_t motorTestIdx;
extern uint8_t motorTestActive;

void Menu_Init(void);
void Menu_Process(ButtonEvents_t *events);
void Menu_Draw(void);

#endif /* __MENU_H */
