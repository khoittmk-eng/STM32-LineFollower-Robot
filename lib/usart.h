#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include <stdio.h>

/* Khai báo các nguyên mẫu hàm (Function Prototypes) */
void USART1_Init(void);
void USART_SendString(USART_TypeDef* USARTx, char* str);

#endif /* __USART_H */