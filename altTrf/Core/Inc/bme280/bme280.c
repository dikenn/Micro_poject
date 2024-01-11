#include "bme280.h"

calibData _bme280_calib;

void BME280(I2C_HandleTypeDef *i2CInst)
{
  _bme280_calib.i2CInst = i2CInst;
  _bme280_calib.t_fine = 0;
  _bme280_calib.t_fine_adjust = 0;
   BME280_begin();
}
void  BME280_begin()
{
   BME280_writeTo(BME280_REGISTER_SOFTRESET, 0xB6);
  HAL_Delay(10);
  int a=0;
  while ( !BME280_isReadingCalibration())
  {
    HAL_Delay(10);
    a++;
    if(a>30){
    	return;
    }
  }
   BME280_readCoefficients();
   BME280_setDefaultSampling();
}
void  BME280_setDefaultSampling()
{

   BME280_writeTo(BME280_REGISTER_CONTROL, MODE_SLEEP);
   BME280_writeTo(BME280_REGISTER_CONTROLHUMID, 0x05);
   BME280_writeTo(BME280_REGISTER_CONFIG, 0);
   BME280_writeTo(BME280_REGISTER_CONTROL, 57);
}

bool  BME280_isReadingCalibration()
{
  uint8_t _b;
   BME280_readFrom(BME280_REGISTER_STATUS, 1, &_b);

  return (_b & (1 << 0)) != 0;
}

void  BME280_readCoefficients(void)
{
  _bme280_calib.dig_T1 = (unsigned short) BME280_readFrom16(BME280_REGISTER_DIG_T1);
  _bme280_calib.dig_T2 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_T2);
  _bme280_calib.dig_T3 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_T3);

  _bme280_calib.dig_P1 = (unsigned short) BME280_readFrom16(BME280_REGISTER_DIG_P1);
  _bme280_calib.dig_P2 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_P2);
  _bme280_calib.dig_P3 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_P3);
  _bme280_calib.dig_P4 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_P4);
  _bme280_calib.dig_P5 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_P5);
  _bme280_calib.dig_P6 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_P6);
  _bme280_calib.dig_P7 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_P7);
  _bme280_calib.dig_P8 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_P8);
  _bme280_calib.dig_P9 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_P9);

  _bme280_calib.dig_H1 = (unsigned char) BME280_readFrom8(BME280_REGISTER_DIG_H1);
  _bme280_calib.dig_H2 = (signed short) BME280_readFrom16(BME280_REGISTER_DIG_H2);
  _bme280_calib.dig_H3 = (unsigned char) BME280_readFrom8(BME280_REGISTER_DIG_H3);
  _bme280_calib.dig_H4 = (signed short)(( BME280_readFrom8(BME280_REGISTER_DIG_H4) << 4) |
                                        ( BME280_readFrom8(BME280_REGISTER_DIG_H4 + 1) & 0x0f));
  _bme280_calib.dig_H5 = (signed short)(( BME280_readFrom8(BME280_REGISTER_DIG_H5 + 1) << 4) |
                                        ( BME280_readFrom8(BME280_REGISTER_DIG_H5) >> 4));
  _bme280_calib.dig_H6 = (signed char) BME280_readFrom8(BME280_REGISTER_DIG_H6);
}

float  BME280_BME280_readPressure(void)
{
   BME280_writeTo(BME280_REGISTER_CONTROL, 57);

  uint8_t _buf[3];
   BME280_readFrom(BME280_REGISTER_TEMPDATA, 3, _buf);
  // printf("%d\n", (_buf[0]));
  // printf("%d\n", (_buf[1]));
  // printf("%d\n", (_buf[2]));

  int32_t adc_T = ((uint32_t)(_buf[0]) << 16) | ((uint32_t)(_buf[1]) << 8) | ((uint32_t)((_buf[2])));
  // printf("adc_T: %d\n", adc_T);
  if (adc_T == 0x800000)
    return NAN;

  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) * ((int32_t)_bme280_calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) * ((int32_t)_bme280_calib.dig_T3)) >> 14;

  _bme280_calib.t_fine = var1 + var2;
  T = (_bme280_calib.t_fine * 5 + 128) >> 8;
  return T;
}

float readPressure(void)
{

   BME280_BME280_readPressure(); // must be done first to get _bme280_calib.t_fine
  uint8_t _buf[3];
   BME280_readFrom(BME280_REGISTER_PRESSUREDATA, 3, _buf);
  int32_t adc_P = ((int32_t)(_buf[0]) << 12) | ((int32_t)(_buf[1]) << 4) | ((int32_t)(_buf[2]));
  if (adc_P == 0x800000) // value in case pressure measurement was disabled
    return NAN;

  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)_bme280_calib.t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)_bme280_calib.dig_P6);
  var2 = var2 + ((var1 * ((int32_t)_bme280_calib.dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)_bme280_calib.dig_P4) << 16);
  var1 = (((_bme280_calib.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)_bme280_calib.dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)_bme280_calib.dig_P1)) >> 15);
  if (var1 == 0)
    return 0;

  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000)
    p = (p << 1) / ((uint32_t)var1);
  else
    p = (p / (uint32_t)var1) * 2;

  var1 = (((int32_t)_bme280_calib.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)_bme280_calib.dig_P8)) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + _bme280_calib.dig_P7) >> 4));

  return p;
}

float  BME280_readHumidity(void)
{
  //int32_t var1, var2, var3, var4, var5;

   BME280_BME280_readPressure(); // must be done first to get _bme280_calib.t_fine
  uint8_t(_buf[2]);
   BME280_readFrom(BME280_REGISTER_HUMIDDATA, 2, _buf);
  int32_t adc_H = (((uint32_t)(_buf[0]) << 8) | ((uint32_t)(_buf[1])));
  if (adc_H == 0x8000) // value in case humidity measurement was disabled
    return NAN;

  int32_t v_x1_u32r;
  v_x1_u32r = (_bme280_calib.t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) - (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) * (((v_x1_u32r *
                                                                               ((int32_t)_bme280_calib.dig_H3)) >>
                                                                              11) +
                                                                             ((int32_t)32768))) >>
                   10) +
                  ((int32_t)2097152)) *
                     ((int32_t)_bme280_calib.dig_H2) +
                 8192) >>
                14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)_bme280_calib.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

  return (uint32_t)(v_x1_u32r >> 12);
}

void  BME280_writeTo(uint8_t address, uint8_t val)
{
  uint8_t data[2] = {address, val};
  HAL_I2C_Master_Transmit(_bme280_calib.i2CInst, BME280_ADDRESS, data, 2, 20);
}

// Reads num uint8_ts starting from address register on device in to _buff array
void  BME280_readFrom(uint8_t address, int num, uint8_t _buff[])
{
  uint8_t data[1] = {address};

  HAL_I2C_Master_Transmit(_bme280_calib.i2CInst, BME280_ADDRESS, data, 2, 20);
  HAL_I2C_Master_Receive(_bme280_calib.i2CInst, BME280_ADDRESS, _buff, num, 20);
}

uint8_t  BME280_readFrom8(uint8_t address)
{
  uint8_t data[1] = {address};
  uint8_t _buff[1];

  HAL_I2C_Master_Transmit(_bme280_calib.i2CInst, BME280_ADDRESS, data, 1, 20);
  HAL_I2C_Master_Receive(_bme280_calib.i2CInst, BME280_ADDRESS, _buff, 1, 20);
  return *_buff;
}
uint16_t  BME280_readFrom16(uint8_t address)
{
  uint8_t data[1] = {address};
  uint8_t _buff[2];

  HAL_I2C_Master_Transmit(_bme280_calib.i2CInst, BME280_ADDRESS, data, 1, 20);
  HAL_I2C_Master_Receive(_bme280_calib.i2CInst, BME280_ADDRESS, _buff, 2, 20);

  return _buff[1] << 8 | _buff[0];
}

void  BME280_read(float *thp)
{

  thp[0] =  BME280_BME280_readPressure();

  thp[1] =  BME280_readHumidity();

  thp[2] = readPressure();
}
void  BME280_PrintRegister()
{

  for (int i = 0; i < 47; i++)
  {
     BME280_readFrom8(0xd0 + i);
  }
}
