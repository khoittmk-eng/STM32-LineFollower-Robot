#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

/* H?m kh?i t?o ADC k?t h?p DMA. 
   Truy?n v?o con tr? c?a m?ng l?u 8 gi? tr? */
void ADC1_DMA_Init(uint16_t* adc_buffer);

#endif /* __ADC_H */
