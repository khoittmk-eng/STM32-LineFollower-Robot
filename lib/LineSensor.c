#include "LineSensor.h"

// Khoi tao cam bien
void LineSensor_Init(LineSensor_t *sensor)
{
    uint8_t i;

    for (i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        sensor->value[i] = 0;
        sensor->sensorPID[i] = 0;
        sensor->black_value[i] = 0;
        sensor->white_value[i] = 4095;
        sensor->threshold[i]   = 2048; // Nguong khoi tao
    }

    sensor->digital = 0x00; // Khoi tao mac dinh Full 0 (nen trang)
    sensor->active_count = 0;
    sensor->last_dir = 0;
}

// Goi 1 lan duy nhat truoc khi bat dau quet Calib
void LineSensor_StartCalib(LineSensor_t *sensor)
{
    uint8_t i;
    for (i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        sensor->black_value[i] = 4095;
        sensor->white_value[i] = 0;
    }
}

// Goi lien tuc trong vong lap khi xe dang xoay quet qua vach
void LineSensor_UpdateCalib(LineSensor_t *sensor, uint16_t raw[LINE_SENSOR_COUNT])
{
    uint8_t i;
    for (i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        uint16_t current_val = 4095 - raw[i];

        // Cap nhat gia tri Trang (Max)
        if (current_val > sensor->white_value[i])
        {
            sensor->white_value[i] = current_val;
        }

        // Cap nhat gia tri Den/Line (Min)
        if (current_val < sensor->black_value[i])
        {
            sensor->black_value[i] = current_val;
        }
    }
}

// Tinh toan nguong Threshold cho tung mat sau khi Calib xong
void LineSensor_FinishCalib(LineSensor_t *sensor)
{
    uint8_t i;
    for (i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (sensor->white_value[i] <= sensor->black_value[i])
        {
            sensor->black_value[i] = 500;
            sensor->white_value[i] = 3500;
        }
        sensor->threshold[i] = (sensor->black_value[i] + sensor->white_value[i]) / 2;
    }
}

// Anh xa gia tri analog vao thang do [MIN, MAX] theo code goc
uint16_t LineSensor_Map(
    uint16_t value,
    uint16_t black,
    uint16_t white
)
{
    uint32_t result;

    if (white <= black)
    {
        return LINE_SENSOR_MIN;
    }

    if (value < black)
    {
        value = black;
    }

    if (value > white)
    {
        value = white;
    }

    result =
        LINE_SENSOR_MIN +
        (
            (uint32_t)(value - black)
            *
            (LINE_SENSOR_MAX - LINE_SENSOR_MIN)
        )
        /
        (white - black);

    return (uint16_t)result;
}

// Doc gia tri Digital:
// Line -> 1, Nen trang -> 0
// Khi ca 8 mat o ngoai nen trang -> Full 0 (00000000 / 0x00)
uint8_t LineSensor_ReadDigital(
    LineSensor_t *sensor,
    uint16_t raw[LINE_SENSOR_COUNT]
)
{
    uint8_t dig = 0;
    uint8_t count = 0;
    uint8_t i;
    uint8_t left_count = 0;
    uint8_t right_count = 0;

    for (i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        sensor->value[i] = 4095 - raw[i];

        // Line ra 1, Nen trang ra 0
        if (sensor->value[i] > sensor->threshold[i])
        {
            // Mat o tren line -> bit = 1
            dig |= (1 << i);
            count++;

            // Dem so mat bat duoc line o nua trai (0..3) va nua phai (4..7)
            if (i < 4)
            {
                left_count++;
            }
            else
            {
                right_count++;
            }
        }
    }

    sensor->digital = dig;
    sensor->active_count = count;

    // Cap nhat huong lech lan gan nhat bat duoc line:
    // So sanh so mat bat duoc line o nua trai va nua phai,
    // ben nao co so mat bat duoc line nhieu hon thi luu huong ve ben do (-1: trai, +1: phai)
    if (dig != 0x00)
    {
        if (left_count > right_count)
        {
            sensor->last_dir = -1; // Ben trai nhieu mat bat line hon -> xoay trai
        }
        else if (right_count > left_count)
        {
            sensor->last_dir = 1;  // Ben phai nhieu mat bat line hon -> xoay phai
        }
        // Neu left_count == right_count: giu nguyen last_dir lan gan nhat truoc do
    }

    return dig;
}

// Tinh toan sai so vi tri line theo thuat toan goc ban dau cua ban
int32_t LineSensor_GetError(
    LineSensor_t *sensor,
    uint16_t raw[LINE_SENSOR_COUNT]
)
{
    uint8_t i;
    uint32_t sum = 0;
    uint32_t weighted_sum = 0;
    int32_t position;
    int32_t error;

    // 1. Cap nhat doc Digital
    LineSensor_ReadDigital(sensor, raw);

    // 2. Chuan hoa sensorPID theo code ban dau cua ban
    for (i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (sensor->value[i] < sensor->black_value[i])
        {
            sensor->value[i] = sensor->black_value[i];
        }

        if (sensor->value[i] > sensor->white_value[i])
        {
            sensor->value[i] = sensor->white_value[i];
        }

        sensor->sensorPID[i] =
            LineSensor_Map(
                sensor->value[i],
                sensor->black_value[i],
                sensor->white_value[i]
            );
    }

    // 3. Tinh vi tri line theo code ban dau cua ban
    for (i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        weighted_sum += (uint32_t)sensor->sensorPID[i] * (uint32_t)(i * 3000);
        sum += sensor->sensorPID[i];
    }

    if (sum == 0)
    {
        return 0;
    }

    position = (int32_t)(weighted_sum / sum);
    error = position - LINE_CENTER;

    // Neu chua tung xac dinh duoc huong (last_dir == 0), ho tro xac dinh ban dau qua error
    if (sensor->last_dir == 0)
    {
        if (error > 400)
        {
            sensor->last_dir = 1;  // Vach o ben phai
        }
        else if (error < -400)
        {
            sensor->last_dir = -1; // Vach o ben trai
        }
    }

    return error;
}
