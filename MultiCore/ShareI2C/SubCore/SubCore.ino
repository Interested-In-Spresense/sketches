/*
 *  SubCore.ino - MultiCore Example for I2C share.
 *  Author Interested-In-Spresense
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
 
#include <MP.h>
#include <cxd56_i2c.h>
#include <Wire.h>

#define MSG_REQ_BMP   1
#define MSG_RET_BMP   2

#define BMP_ADDR        0x76
#define REG_ID          0xD0
#define REG_RESET       0xE0
#define REG_CTRL_MEAS   0xF4
#define REG_CONFIG      0xF5
#define REG_PRESS_MSB   0xF7

// ------------------------------------------------------
// I2C Write Helper
// ------------------------------------------------------
bool bmp_write_reg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

// ------------------------------------------------------
// I2C Read Helper
// ------------------------------------------------------
bool bmp_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    Serial.printf("[SUB] Write reg 0x%02X failed\n", reg);
    return false;
  }

  if (Wire.requestFrom(BMP_ADDR, len) != len) {
    Serial.printf("[SUB] Read reg 0x%02X failed\n", reg);
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

// ------------------------------------------------------
// BMP280 Init
// ------------------------------------------------------
bool bmp_init_raw()
{
  Wire.begin();
  Wire.setClock(400000);
  delay(10);

  uint8_t id = 0;
  if (!bmp_read_bytes(REG_ID, &id, 1)) {
    Serial.println("[SUB] ID read fail");
    return false;
  }

  Serial.printf("[SUB] BMP ID=0x%02X\n", id);

  // Soft reset
  bmp_write_reg(REG_RESET, 0xB6);
  delay(200);

  if (!bmp_read_bytes(REG_ID, &id, 1)) {
    Serial.println("[SUB] ID read after reset fail");
  } else {
    Serial.printf("[SUB] BMP ID(after reset)=0x%02X\n", id);
  }

  bmp_write_reg(REG_CONFIG, 0x10);
  bmp_write_reg(REG_CTRL_MEAS, 0x27);
  delay(50);

  Serial.println("[SUB] BMP RAW init OK");
  return true;
}

// ------------------------------------------------------
// Read raw 20-bit pressure
// ------------------------------------------------------
int32_t bmp_read_raw_pressure()
{
  uint8_t buf[3];
  if (!bmp_read_bytes(REG_PRESS_MSB, buf, 3)) {
    return -1;
  }

  uint32_t raw =
    ((uint32_t)buf[0] << 12) |
    ((uint32_t)buf[1] << 4 ) |
    ((uint32_t)buf[2] >> 4);

  return (int32_t)raw;
}

// ------------------------------------------------------
// Setup
// ------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("== SubCore BMP280 RAW HAL Mode ==");
  MP.begin();
  MP.RecvTimeout(3000);
}

// ------------------------------------------------------
// Loop (wait for main → measure → return)
// ------------------------------------------------------
void loop()
{
  int8_t msgid;
  uint32_t dummy;

  if (MP.Recv(&msgid, &dummy) >= 0 && msgid == MSG_REQ_BMP) {

    if (!bmp_init_raw()) {
      Serial.println("[SUB] bmp_init_raw failed");
    }

    int32_t raw = bmp_read_raw_pressure();

    if (raw < 0) {
      Serial.println("[SUB] RAW read fail");
      Wire.end();
    } else {
      float hpa = (raw / 256.0f) / 1.33f;
      Serial.printf("[SUB] RAW=%ld  →  ~%.2f hPa\n", (long)raw, hpa);
      Wire.end();
      MP.Send(MSG_RET_BMP, hpa);
    }

  } else {
    Serial.println("[Sub] recv timeout");
  }

}
