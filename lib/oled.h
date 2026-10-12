#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"

/* ==================================================================== */
/*                C?U H?NH PH?N C?NG (D? D?NG CH?NH S?A)                */
/* ==================================================================== */
#define OLED_I2C_PORT           I2C1
#define OLED_I2C_CLK            RCC_APB1Periph_I2C1
#define OLED_I2C_CLK_CMD        RCC_APB1PeriphClockCmd  

#define OLED_GPIO_PORT          GPIOB
#define OLED_GPIO_CLK           RCC_APB2Periph_GPIOB
#define OLED_GPIO_CLK_CMD       RCC_APB2PeriphClockCmd  

#define OLED_SCL_PIN            GPIO_Pin_8
#define OLED_SDA_PIN            GPIO_Pin_9
#define OLED_I2C_ADDR           0x78

#define OLED_WIDTH              128
#define OLED_HEIGHT             64

/* ==================================================================== */
/*                       KHAI B?O C?C H?M GIAO TI?P                     */
/* ==================================================================== */
void SH1106_Init(void);
void SH1106_UpdateScreen(void);
void SH1106_Clear(void);
void SH1106_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_DrawChar(uint8_t x, uint8_t y, char c, uint8_t color);
void OLED_DrawString(uint8_t x, uint8_t y, const char *str, uint8_t color);
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void OLED_TestDeadPixel(void);

#endif // __SH1106_H
