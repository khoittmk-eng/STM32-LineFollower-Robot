#include "adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h" // Thêm thư viện DMA

void ADC1_DMA_Init(uint16_t* adc_buffer) {
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // 1. Cấp clock cho GPIOA, ADC1 và đặc biệt là DMA1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // Bật Clock cho DMA1

    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // Chia clock ADC (12MHz)

    // 2. Cấu hình PA0 -> PA7 là Analog Input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. Cấu hình DMA1 - Kênh 1 (Vì ADC1 kết nối cứng với DMA1 Channel 1)
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; // Lấy data từ thanh ghi này
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adc_buffer;    // Đổ data vào mảng của bạn
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;              // Hướng: Ngoại vi -> Bộ nhớ
    DMA_InitStructure.DMA_BufferSize = 8;                           // Quét đủ 8 kênh
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;// Thanh ghi ADC không tăng
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         // Địa chỉ mảng bộ nhớ tăng dần
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16-bit
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // 16-bit
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                 // Chế độ xoay vòng liên tục
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    DMA_Cmd(DMA1_Channel1, ENABLE); // Kích hoạt DMA

    // 4. Cấu hình ADC1 ở chế độ SCAN (Quét) và CONTINUOUS (Liên tục)
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;           // BẬT quét nhiều kênh
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;     // BẬT chuyển đổi liên tục
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 8;                // Đọc 8 kênh
    ADC_Init(ADC1, &ADC_InitStructure);

    // Xếp hạng (Rank) thứ tự đọc các kênh từ 1 đến 8
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_55Cycles5);

    // Kích hoạt tính năng DMA của ADC
    ADC_DMACmd(ADC1, ENABLE);

    // 5. Bật ADC1 và Hiệu chỉnh
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    // Kích hoạt lần chuyển đổi đầu tiên (Các lần sau nó tự động lặp lại nhờ ContinuousMode)
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}