
#include <stdio.h>
#include "math.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#ifndef BME280_H_
#define BME280_H_
#define BME280_ADDRESS 0xEC

#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_T2 0x8A
#define BME280_REGISTER_DIG_T3 0x8C

#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_P2 0x90
#define BME280_REGISTER_DIG_P3 0x92
#define BME280_REGISTER_DIG_P4 0x94
#define BME280_REGISTER_DIG_P5 0x96
#define BME280_REGISTER_DIG_P6 0x98
#define BME280_REGISTER_DIG_P7 0x9A
#define BME280_REGISTER_DIG_P8 0x9C
#define BME280_REGISTER_DIG_P9 0x9E

#define BME280_REGISTER_DIG_H1 0xA1
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_DIG_H3 0xE3
#define BME280_REGISTER_DIG_H4 0xE4
#define BME280_REGISTER_DIG_H5 0xE5
#define BME280_REGISTER_DIG_H6 0xE7

#define BME280_REGISTER_CHIPID 0xD0
#define BME280_REGISTER_VERSION 0xD1
#define BME280_REGISTER_SOFTRESET 0xE0

#define BME280_REGISTER_CAL26 0xE1 // R calibration stored in 0xE1-0xF0

#define BME280_REGISTER_CONTROLHUMID 0xF2
#define BME280_REGISTER_STATUS 0XF3
#define BME280_REGISTER_CONTROL 0xF4
#define BME280_REGISTER_CONFIG 0xF5
#define BME280_REGISTER_PRESSUREDATA 0xF7
#define BME280_REGISTER_TEMPDATA 0xFA
#define BME280_REGISTER_HUMIDDATA 0xFD

#define SAMPLING_NONE 0b000
#define SAMPLING_X1 0b001
#define SAMPLING_X2 0b010
#define SAMPLING_X4 0b011
#define SAMPLING_X8 0b100
#define SAMPLING_X16 0b101

#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11

#define FILTER_OFF 0b000
#define FILTER_X2 0b001
#define FILTER_X4 0b010
#define FILTER_X8 0b011
#define FILTER_X16 0b100

#define STANDBY_MS_0_5 0b000
#define STANDBY_MS_10 0b110
#define STANDBY_MS_20 0b111
#define STANDBY_MS_62_5 0b001
#define STANDBY_MS_125 0b010
#define STANDBY_MS_250 0b011
#define STANDBY_MS_500 0b100
#define STANDBY_MS_1000 0b101
typedef struct calibData
{
    uint16_t dig_T1; ///< temperature compensation value
    int16_t dig_T2;  ///< temperature compensation value
    int16_t dig_T3;  ///< temperature compensation value

    uint16_t dig_P1; ///< pressure compensation value
    int16_t dig_P2;  ///< pressure compensation value
    int16_t dig_P3;  ///< pressure compensation value
    int16_t dig_P4;  ///< pressure compensation value
    int16_t dig_P5;  ///< pressure compensation value
    int16_t dig_P6;  ///< pressure compensation value
    int16_t dig_P7;  ///< pressure compensation value
    int16_t dig_P8;  ///< pressure compensation value
    int16_t dig_P9;  ///< pressure compensation value

    uint8_t dig_H1; ///< humidity compensation value
    int16_t dig_H2; ///< humidity compensation value
    uint8_t dig_H3; ///< humidity compensation value
    int16_t dig_H4; ///< humidity compensation value
    int16_t dig_H5; ///< humidity compensation value
    int8_t dig_H6;  ///< humidity compensation value
    int32_t t_fine;
    int32_t t_fine_adjust;
    I2C_HandleTypeDef *i2CInst;
} calibData;

void BME280(I2C_HandleTypeDef *i2CInst);
void BME280_begin();
void BME280_PrintRegister();
bool BME280_isReadingCalibration();
float BME280_readTemperature();
float BME280_readPressure();
float BME280_readHumidity();
void BME280_setDefaultSampling();
void BME280_read(float *thp);
void BME280_readCoefficients();
uint8_t BME280_readFrom8(uint8_t address);
uint16_t BME280_readFrom16(uint8_t address);
void BME280_writeTo(uint8_t address, uint8_t val);
void BME280_readFrom(uint8_t address, int num, uint8_t _buff[]);
#endif