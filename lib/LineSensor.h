#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#include "stm32f10x.h"

#define LINE_SENSOR_COUNT    8

#define LINE_SENSOR_MIN      1
#define LINE_SENSOR_MAX      3000

#define LINE_CENTER          10500

typedef struct
{
    uint16_t value[LINE_SENSOR_COUNT];
    uint16_t sensorPID[LINE_SENSOR_COUNT];
    uint16_t black_value[LINE_SENSOR_COUNT];
    uint16_t white_value[LINE_SENSOR_COUNT];
    uint16_t threshold[LINE_SENSOR_COUNT]; // Nguong phan dinh giua Den va Trang

    uint8_t  digital;        // Bitmask 8-bit (bit 0..7: 1 = Line, 0 = Nen trang -> Full 0 la ra nen trang)
    uint8_t  active_count;   // So luong mat tren vach line (so bit 1, tu 0..8)
    int8_t   last_dir;       // Huong lech gan nhat (-1: trai, +1: phai)

} LineSensor_t;

void LineSensor_Init(LineSensor_t *sensor);
void LineSensor_StartCalib(LineSensor_t *sensor);
void LineSensor_UpdateCalib(LineSensor_t *sensor, uint16_t raw[LINE_SENSOR_COUNT]);
void LineSensor_FinishCalib(LineSensor_t *sensor);

uint16_t LineSensor_Map(
    uint16_t value,
    uint16_t black,
    uint16_t white
);

uint8_t LineSensor_ReadDigital(
    LineSensor_t *sensor,
    uint16_t raw[LINE_SENSOR_COUNT]
);

int32_t LineSensor_GetError(
    LineSensor_t *sensor,
    uint16_t raw[LINE_SENSOR_COUNT]
);

#endif /* __LINE_SENSOR_H */
