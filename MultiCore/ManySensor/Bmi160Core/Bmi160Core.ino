/*
 *  Bmi160Core.ino - BMI160 sensor core for ManySensor.
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

#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <BMI160Gen.h>

#define MSG_REQ_BMI   3
#define MSG_RET_BMI   4

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // initialize device
  BMI160.begin();

  // Set the accelerometer range to 2G
  BMI160.setAccelerometerRange(2);
  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  MP.begin();
  MP.RecvTimeout(3000);
}

// ------------------------------------------------------------
// LOOP (wait for main → measure → return)
// ------------------------------------------------------------
void loop() {

  int8_t msgid;
  uint32_t dummy;

  if (MP.Recv(&msgid, &dummy) >= 0 && msgid == MSG_REQ_BMI) {

    float ax, ay, az;   //scaled accelerometer values

    up_enable_irq(CXD56_IRQ_SCU_I2C0);

    // read accelerometer measurements from device, scaled to the configured range
    BMI160.readAccelerometerScaled(ax, ay, az);

    // display tab-separated accelerometer x/y/z values
    Serial.print("[Sub2] acc:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.println();
    Serial.flush();

    up_disable_irq(CXD56_IRQ_SCU_I2C0);
    MP.Send(MSG_RET_BMI, (uint32_t)0);

  } else {
    Serial.println("[Sub2] recv timeout");
  }
}
