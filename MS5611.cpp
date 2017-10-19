/*
  MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.

  Version: 1.0.0
  (c) 2014 Korneliusz Jarzebski
  www.jarzebski.pl

  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <math.h>

#include "MS5611.h"

bool MS5611::begin(ms5611_osr_t osr)
{
  Wire.begin();

  reset();

  setOversampling(osr);

  delay(100);

  readPROM();

  return true;
}

// Set oversampling value
void MS5611::setOversampling(ms5611_osr_t osr)
{
  switch (osr)
  {
    case MS5611_ULTRA_LOW_POWER:
      ct = 1;
      break;
    case MS5611_LOW_POWER:
      ct = 2;
      break;
    case MS5611_STANDARD:
      ct = 3;
      break;
    case MS5611_HIGH_RES:
      ct = 5;
      break;
    case MS5611_ULTRA_HIGH_RES:
      ct = 10;
      break;
  }

  uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611::getOversampling(void)
{
  return (ms5611_osr_t)uosr;
}

void MS5611::reset(void)
{
  Wire.beginTransmission(MS5611_ADDRESS);

#if ARDUINO >= 100
  Wire.write(MS5611_CMD_RESET);
#else
  Wire.send(MS5611_CMD_RESET);
#endif

  Wire.endTransmission();
}

void MS5611::readPROM(void)
{
  for (uint8_t offset = 0; offset < 6; offset++)
  {
    fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
  }
}

uint32_t MS5611::readRawTemperature(void)
{
  Wire.beginTransmission(MS5611_ADDRESS);

#if ARDUINO >= 100
  Wire.write(MS5611_CMD_CONV_D2 + uosr);
#else
  Wire.send(MS5611_CMD_CONV_D2 + uosr);
#endif

  Wire.endTransmission();

  delay(ct);

  return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t MS5611::readRawPressure(void)
{
  Wire.beginTransmission(MS5611_ADDRESS);

#if ARDUINO >= 100
  Wire.write(MS5611_CMD_CONV_D1 + uosr);
#else
  Wire.send(MS5611_CMD_CONV_D1 + uosr);
#endif

  Wire.endTransmission();

  delay(ct);

  return readRegister24(MS5611_CMD_ADC_READ);
}

int32_t MS5611::readPressureFast()
{
  uint32_t D1 = readRawPressure();

  int64_t OFF = (int64_t)fc[1] * 65536;
  int64_t SENS = (int64_t)fc[0] * 32768;

  uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

  return P;
}

int32_t MS5611::readPressure(bool compensation)
{
  uint32_t D1 = readRawPressure();

  uint32_t D2 = readRawTemperature();
  int32_t dT = D2 - (uint32_t)fc[4] * 256;

  int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
  int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

  if (compensation)
  {
    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    OFF2 = 0;
    SENS2 = 0;

    if (TEMP < 2000)
    {
      OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
      SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
    }

    if (TEMP < -1500)
    {
      OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
      SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
    }

    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
  }

  uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

  return P;
}

double MS5611::readTemperature(bool compensation)
{
  uint32_t D2 = readRawTemperature();
  int32_t dT = D2 - (uint32_t)fc[4] * 256;

  int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

  TEMP2 = 0;

  if (compensation)
  {
    if (TEMP < 2000)
    {
      TEMP2 = (dT * dT) / (2 << 30);
    }
  }

  TEMP = TEMP - TEMP2;

  return ((double)TEMP / 100);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::getAltitude(double pressure, double seaLevelPressure)
{
  return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double MS5611::getSeaLevel(double pressure, double altitude)
{
  return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611::readRegister16(uint8_t reg)
{
  uint16_t value;
  Wire.beginTransmission(MS5611_ADDRESS);
#if ARDUINO >= 100
  Wire.write(reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (!Wire.available()) {};
#if ARDUINO >= 100
  uint8_t vha = Wire.read();
  uint8_t vla = Wire.read();
#else
  uint8_t vha = Wire.receive();
  uint8_t vla = Wire.receive();
#endif
  Wire.endTransmission();

  value = vha << 8 | vla;

  return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611::readRegister24(uint8_t reg)
{
  uint32_t value;
  Wire.beginTransmission(MS5611_ADDRESS);
#if ARDUINO >= 100
  Wire.write(reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.requestFrom(MS5611_ADDRESS, 3);
  while (!Wire.available()) {};
#if ARDUINO >= 100
  uint8_t vxa = Wire.read();
  uint8_t vha = Wire.read();
  uint8_t vla = Wire.read();
#else
  uint8_t vxa = Wire.receive();
  uint8_t vha = Wire.receive();
  uint8_t vla = Wire.receive();
#endif
  Wire.endTransmission();

  value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

  return value;
}
