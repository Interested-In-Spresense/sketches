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
#include <BMI160Gen.h>

#define SUBCORE_ID  1
#define MSG_REQ_BMP   1
#define MSG_RET_BMP   2

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

  MP.begin(SUBCORE_ID);
  MP.RecvTimeout(3000);

}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {

  Serial.println("\n[Main] → BMP request sent");
  int ret = MP.Send(MSG_REQ_BMP, (uint32_t)0, SUBCORE_ID);
  Serial.flush();

  int8_t msgid;
  uint32_t bmp_raw = 0;

  if (MP.Recv(&msgid, &bmp_raw, SUBCORE_ID) >= 0 && msgid == MSG_RET_BMP) {

    float ax, ay, az;   //scaled accelerometer values

    up_enable_irq(CXD56_IRQ_SCU_I2C0);

    // read accelerometer measurements from device, scaled to the configured range
    BMI160.readAccelerometerScaled(ax, ay, az);

    // display tab-separated accelerometer x/y/z values
    Serial.print("[Main] acc:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.println();
    Serial.flush();

    up_disable_irq(CXD56_IRQ_SCU_I2C0);

  } else {
    Serial.println("[Main] No BMP reply");
  }
}
