/*
 *  MainCore.ino - MultiCore Example for I2C share.
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
#include <errno.h>
#include <Wire.h>

#define SUBCORE_ID  1
#define MSG_REQ_BMP   1
#define MSG_RET_BMP   2

#define BMI160_ADDR    0x68
#define REG_CMD        0x7E
#define REG_ACC_CONF   0x40
#define BMI160_ACC_REG 0x12

// ------------------------------------------------------------
// BMI160 INIT
// ------------------------------------------------------------
bool bmi160_init()
{
  Serial.println("[BMI160] Init via Wire");

  Wire.begin();
  Wire.setClock(400000);

  // ---- 1) Soft reset ----
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_CMD);     // CMD register
  Wire.write(0xB6);        // Soft reset
  if (Wire.endTransmission() != 0) {
    Serial.println("[BMI160] Soft reset failed");
    return false;
  }
  delay(10);

  // ---- 2) Accelerometer normal mode ----
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_CMD);     // CMD
  Wire.write(0x11);        // ACC normal mode
  if (Wire.endTransmission() != 0) {
    Serial.println("[BMI160] ACC normal mode set fail");
    return false;
  }
  delay(50);

  // ---- 3) ODR = 100Hz ----
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x41);       // ACC_CONF (ODR/reg)
  Wire.write(0x0D);       // ODR = 100Hz
  if (Wire.endTransmission() != 0) {
    Serial.println("[BMI160] Set ODR fail");
    return false;
  }

  // ---- 4) Range = ±2G ----
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x40);       // ACC_RANGE
  Wire.write(0x03);       // ±2G
  if (Wire.endTransmission() != 0) {
    Serial.println("[BMI160] Set range fail");
    return false;
  }

  Serial.println("[BMI160] Wire init completed");
  Wire.end();
  return true;
}

// ------------------------------------------------------------
// BMI160 READ ACC
// ------------------------------------------------------------
bool read_bmi160_acc(float *x, float *y, float *z)
{
  uint8_t data[6];

  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(BMI160_ACC_REG);    // start register
  if (Wire.endTransmission(false) != 0) {
    Serial.println("[BMI160] ACC reg write fail");
    return false;
  }

  if (Wire.requestFrom(BMI160_ADDR, 6) != 6) {
    Serial.println("[BMI160] ACC read fail");
    return false;
  }

  for (int i = 0; i < 6; i++) data[i] = Wire.read();

  int16_t rawX = (data[1] << 8) | data[0];
  int16_t rawY = (data[3] << 8) | data[2];
  int16_t rawZ = (data[5] << 8) | data[4];

  *x = rawX * 0.000061f;
  *y = rawY * 0.000061f;
  *z = rawZ * 0.000061f;

  Wire.end();
  return true;
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("== Main HAL Debug Mode ==");

  MP.begin(SUBCORE_ID);
  MP.RecvTimeout(3000);

}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {

  Serial.println("\n[Main] → BMP request sent");
  int ret = MP.Send(MSG_REQ_BMP, (uint32_t)0, SUBCORE_ID);
  Serial.printf("[Main] MP.Send ret=%d\n", ret);

  int8_t msgid;
  uint32_t bmp_raw = 0;

  if (MP.Recv(&msgid, &bmp_raw, SUBCORE_ID) >= 0 && msgid == MSG_RET_BMP) {

    bmi160_init();

    float ax, ay, az;
    read_bmi160_acc(&ax, &ay, &az);

    Serial.printf("[MAIN] ACC = %.3f, %.3f, %.3f g\n", ax, ay, az);
    Serial.println("------------------------------------------------");

  } else {
    Serial.println("[Main] No BMP reply");
  }
}
