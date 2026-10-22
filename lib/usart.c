#include "usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

// Hàm khởi tạo và cấu hình USART1
void USART1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // Cấp clock cho GPIOA, USART1 và AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    // PA9 (TX) - Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 (RX) - Input Floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Cấu hình Baudrate 115200, 8-bit, 1 stop bit, No parity
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    // Bật USART1
    USART_Cmd(USART1, ENABLE);
}

// Hàm gửi chuỗi trực tiếp (Không phụ thuộc printf)
void USART_SendString(USART_TypeDef* USARTx, char* str) {
    while(*str) {
        USART_SendData(USARTx, *str++);
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
    }
}

// ========================================================
// Retarget lại hàm fputc để sử dụng được printf qua USART1
// ========================================================
struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
    USART_SendData(USART1, (uint8_t) ch);
    // Chờ cho đến khi gửi xong
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    return ch;
}
