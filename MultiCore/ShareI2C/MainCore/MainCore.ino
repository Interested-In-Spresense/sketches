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
#include <cxd56_i2c.h>
#include <errno.h>

#define SUBCORE_ID    1
#define MSG_REQ_BMP   1
#define MSG_RET_BMP   2

#define BMI160_ADDR    0x68
#define REG_CMD        0x7E
#define REG_ACC_CONF   0x40
#define BMI160_ACC_REG 0x12

static uint8_t i2c_buf[6];

// ------------------------------------------------------------
// BMI160 INIT
// ------------------------------------------------------------
bool bmi160_init_hal() {

  struct i2c_master_s *dev = cxd56_i2cbus_initialize(0);
  if (!dev) return false;

  struct i2c_msg_s msg;
  uint8_t data[2];

  // ---- 1) Soft reset ----
  data[0] = 0x7E;
  data[1] = 0xB6;
  msg.addr      = BMI160_ADDR;
  msg.flags     = 0;
  msg.buffer    = data;
  msg.length    = 2;
  msg.frequency = 400000;
  I2C_TRANSFER(dev, &msg, 1);
  delay(10);

  // ---- 2) Enable accelerometer (normal mode) ----
  data[0] = 0x7E;
  data[1] = 0x11;
  I2C_TRANSFER(dev, &msg, 1);
  delay(50);

  // ---- 3) Set ODR = 100Hz ----
  data[0] = 0x41;
  data[1] = 0x0D;
  I2C_TRANSFER(dev, &msg, 1);

  // ---- 4) Set range = ±2G ----
  data[0] = 0x40;
  data[1] = 0x03;
  I2C_TRANSFER(dev, &msg, 1);

  cxd56_i2cbus_uninitialize(dev);

  Serial.println("[BMI160] FULL HAL init completed");

  return true;
}

#define BMI160_WHOAMI   0x00
#define EXPECTED_ID     0xD1   // BMI160 chip ID

// ------------------------------------------------------------
// BMI160 WHOAMI
// ------------------------------------------------------------
bool bmi160_wakeup() {

  Serial.println("[BMI160] bmi160_wakeup() enter");

  struct i2c_master_s *dev = cxd56_i2cbus_initialize(0);
  if (!dev) {
    Serial.println("[BMI160] I2C init fail");
    return false;
  }

  uint8_t reg = BMI160_WHOAMI;
  uint8_t val = 0;

  struct i2c_msg_s msg[2];

  // ---- msg[0] Write reg ----
  msg[0].addr      = BMI160_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;
  msg[0].frequency = 400000;

  // ---- msg[1] Read response ----
  msg[1].addr      = BMI160_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &val;
  msg[1].length    = 1;
  msg[1].frequency = 400000;

  uint32_t t = millis();
  while (millis() - t < 50) { // retry max 50ms
    int ret = I2C_TRANSFER(dev, msg, 2);

    if (ret >= 0) {
      Serial.printf("[BMI160] WHOAMI read=0x%02X\n", val);
      if (val == EXPECTED_ID) {
        Serial.println("[BMI160] ✔ WHOAMI MATCH -> I2C MODE READY");
        cxd56_i2cbus_uninitialize(dev);
        return true;
      }
    }
    delay(2);
  }

  Serial.printf("[BMI160] WHOAMI FAIL (last=0x%02X)\n", val);

  cxd56_i2cbus_uninitialize(dev);
  return false;
}

// ------------------------------------------------------------
// BMI160 READ ACC
// ------------------------------------------------------------
bool read_bmi160_acc(float *x, float *y, float *z) {

  struct i2c_master_s *dev = cxd56_i2cbus_initialize(0);
  if (!dev) return false;

  uint8_t reg = BMI160_ACC_REG;

  struct i2c_msg_s msg[2];

  // ---- Write register ----
  msg[0].addr      = BMI160_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;
  msg[0].frequency = 400000;

  // ---- Read 6 bytes ----
  msg[1].addr      = BMI160_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = i2c_buf;
  msg[1].length    = 6;
  msg[1].frequency = 400000;

  int ret = I2C_TRANSFER(dev, msg, 2);
  cxd56_i2cbus_uninitialize(dev);

  if (ret < 0) {
    Serial.printf("[BMI160] READ FAILED ret=%d\n", ret);
    return false;
  }

  int16_t rawX = (i2c_buf[1] << 8) | i2c_buf[0];
  int16_t rawY = (i2c_buf[3] << 8) | i2c_buf[2];
  int16_t rawZ = (i2c_buf[5] << 8) | i2c_buf[4];

  *x = rawX * 0.000061f;
  *y = rawY * 0.000061f;
  *z = rawZ * 0.000061f;

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

  if (!bmi160_wakeup()) {
    Serial.println("[ERR] BMI160 did not enter I2C mode");
    return;
  }

  bmi160_init_hal();

  float ax, ay, az;
  read_bmi160_acc(&ax, &ay, &az);

  Serial.printf("[MAIN] ACC = %.3f, %.3f, %.3f g\n", ax, ay, az);
  Serial.println("------------------------------------------------");
  
  } else {
    Serial.println("[Main] ⚠ No BMP reply");
  }
}
