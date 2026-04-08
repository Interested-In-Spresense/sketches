/*
 *  MainCore.ino - MultiCore coordinator for BMP280/BMI160/MultiIMU sensors.
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

#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <MP.h>

#define BMI160CORE_ID  1
#define BMP280CORE_ID  2
#define M_IMUCORE_ID   3
#define MSG_REQ_BMP   1
#define MSG_RET_BMP   2
#define MSG_REQ_BMI   3
#define MSG_RET_BMI   4
#define MSG_REQ_MIMU  5
#define MSG_RET_MIMU  6

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  MP.begin(BMI160CORE_ID);
  MP.begin(BMP280CORE_ID);
  MP.begin(M_IMUCORE_ID);
  MP.RecvTimeout(3000);
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {

  Serial.println("\n[Main] → BMP request sent");
  int ret = MP.Send(MSG_REQ_BMP, (uint32_t)0, BMP280CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send failed: ");
    Serial.println(ret);
  }
  Serial.flush();

  int8_t msgid;
  uint32_t bmp_raw = 0;

  if (MP.Recv(&msgid, &bmp_raw, BMP280CORE_ID) >= 0 && msgid == MSG_RET_BMP) {
    Serial.println("[Main] BMP reply received");
  } else {
    Serial.println("[Main] No BMP reply");
  }

  Serial.println("[Main] → BMI request sent");
  ret = MP.Send(MSG_REQ_BMI, (uint32_t)0, BMI160CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(BMI) failed: ");
    Serial.println(ret);
  }
  Serial.flush();

  uint32_t bmi_raw = 0;
  if (MP.Recv(&msgid, &bmi_raw, BMI160CORE_ID) >= 0 && msgid == MSG_RET_BMI) {
    Serial.println("[Main] BMI reply received");
  } else {
    Serial.println("[Main] No BMI reply");
  }

  Serial.println("[Main] -> MIMU request sent");
  ret = MP.Send(MSG_REQ_MIMU, (uint32_t)0, M_IMUCORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(MIMU) failed: ");
    Serial.println(ret);
  }
  Serial.flush();

  uint32_t mimu_raw = 0;
  if (MP.Recv(&msgid, &mimu_raw, M_IMUCORE_ID) >= 0 && msgid == MSG_RET_MIMU) {
    Serial.println("[Main] MIMU reply received");
  } else {
    Serial.println("[Main] No MIMU reply");
  }
}
