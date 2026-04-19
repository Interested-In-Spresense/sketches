/*
 *  MainCore.ino - MultiCore coordinator for many sensors.
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

#include <Arduino.h>
#include <MP.h>
#include "ManySensorShared.h"
#include "SharedMemoryAllocator.h"

static SharedMemoryRegion g_shared_memory = {
  { NULL, BMP280_DATA_LEN * sizeof(float), 0, NULL, 0 },
  { NULL, BMI160_DATA_LEN * sizeof(float), 0, NULL, 0 },
  { NULL, MULTIMU_DATA_LEN * sizeof(float), 0, NULL, 0 },
};

constexpr uint32_t MODE_SWITCH_INTERVAL_MS = 10000;
constexpr uint32_t READY_HOLD_MS = 300;
static uint32_t g_bmp_mode = BMP280_MODE_FULL;
static uint32_t g_bmi_mode = BMI160_MODE_ACC;
static uint32_t g_mimu_mode = MIMU_MODE_DATA;
static SharedMemoryLayout g_active_layout = SHARED_LAYOUT_0;

static ShardMemoryAllocator g_allocator;
static bool sendAllSharedAddresses(const SharedMemoryRegion &region);

enum MainState {
  MAIN_STATE_RUN = 0,
  MAIN_STATE_READY,
};

namespace {

void applyModesForLayout(SharedMemoryLayout layout)
{
  if (layout == SHARED_LAYOUT_1) {
    g_bmp_mode = BMP280_MODE_PRESS;
    g_bmi_mode = BMI160_MODE_FULL;
    g_mimu_mode = MIMU_MODE_DATA;
  } else if (layout == SHARED_LAYOUT_2) {
    g_bmp_mode = BMP280_MODE_FULL;
    g_bmi_mode = BMI160_MODE_FULL;
    g_mimu_mode = MIMU_MODE_FULL;
  } else {
    g_bmp_mode = BMP280_MODE_FULL;
    g_bmi_mode = BMI160_MODE_ACC;
    g_mimu_mode = MIMU_MODE_DATA;
  }
}

SharedMemoryLayout nextLayout(SharedMemoryLayout current)
{
  switch (current) {
  case SHARED_LAYOUT_0:
    return SHARED_LAYOUT_1;
  case SHARED_LAYOUT_1:
    return SHARED_LAYOUT_2;
  case SHARED_LAYOUT_2:
  default:
    return SHARED_LAYOUT_0;
  }
}

void printBMP(const float *bmpData)
{
  if (g_bmp_mode == BMP280_MODE_PRESS) {
    Serial.print("[BMP] pressure=");
    Serial.print(bmpData[0]);
    Serial.println("Pa");
  } else {
    Serial.print("[BMP] temp=");
    Serial.print(bmpData[0]);
    Serial.print("C pressure=");
    Serial.print(bmpData[1]);
    Serial.print("Pa altitude=");
    Serial.print(bmpData[2]);
    Serial.println("m");
  }
}

void printBMI(const float *bmiData)
{
  if (g_bmi_mode == BMI160_MODE_ACC) {
    Serial.print("[BMI] acc[m/s^2]=");
    Serial.print(bmiData[0]);
    Serial.print(", ");
    Serial.print(bmiData[1]);
    Serial.print(", ");
    Serial.println(bmiData[2]);
  } else if (g_bmi_mode == BMI160_MODE_GYRO) {
    Serial.print("[BMI] gyro[rad/s]=");
    Serial.print(bmiData[0], 4);
    Serial.print(", ");
    Serial.print(bmiData[1], 4);
    Serial.print(", ");
    Serial.println(bmiData[2], 4);
  } else {
    Serial.print("[BMI] acc[m/s^2]=");
    Serial.print(bmiData[0]);
    Serial.print(", ");
    Serial.print(bmiData[1]);
    Serial.print(", ");
    Serial.print(bmiData[2]);
    Serial.print(" gyro[rad/s]=");
    Serial.print(bmiData[3], 4);
    Serial.print(", ");
    Serial.print(bmiData[4], 4);
    Serial.print(", ");
    Serial.println(bmiData[5], 4);
  }
}

void printMIMU(const float *mimuData)
{
  if (g_mimu_mode == MIMU_MODE_FULL) {
    Serial.print("[MIMU] ts=");
    Serial.print(mimuData[0]);
    Serial.print(" temp[C]=");
    Serial.print(mimuData[1]);
    Serial.print(" acc[m/s^2]=");
    Serial.print(mimuData[2]);
    Serial.print(", ");
    Serial.print(mimuData[3]);
    Serial.print(", ");
    Serial.print(mimuData[4]);
    Serial.print(" gyro[rad/s]=");
    Serial.print(mimuData[5], 4);
    Serial.print(", ");
    Serial.print(mimuData[6], 4);
    Serial.print(", ");
    Serial.println(mimuData[7], 4);
  } else {
    Serial.print("[MIMU] acc[m/s^2]=");
    Serial.print(mimuData[0]);
    Serial.print(", ");
    Serial.print(mimuData[1]);
    Serial.print(", ");
    Serial.print(mimuData[2]);
    Serial.print(" gyro[rad/s]=");
    Serial.print(mimuData[3], 4);
    Serial.print(", ");
    Serial.print(mimuData[4], 4);
    Serial.print(", ");
    Serial.println(mimuData[5], 4);
  }
}

bool waitStateAckFromCore(int coreId, const char *coreName)
{
  int8_t ackMsgid;
  uint32_t ackValue;

  for (int i = 0; i < 4; ++i) {
    int ret = MP.Recv(&ackMsgid, &ackValue, coreId);
    if (ret < 0) {
      Serial.print("[Main] ACK timeout from ");
      Serial.println(coreName);
      return false;
    }
    if (ackMsgid == MSG_RET_STATE) {
      return true;
    }

    /* Drop stale replies (e.g. delayed MSG_RET_BMP/MSG_RET_BMI/MSG_RET_MIMU) */
    Serial.print("[Main] Dropped stale msg from ");
    Serial.print(coreName);
    Serial.print(": msgid=");
    Serial.print(ackMsgid);
    Serial.print(" value=");
    Serial.println(ackValue);
  }

  Serial.print("[Main] ACK not received from ");
  Serial.println(coreName);
  return false;
}

void drainPendingFromCore(int coreId, const char *coreName)
{
  int8_t msgid;
  uint32_t value;

  /* Temporarily shorten timeout to flush queued messages without long blocking */
  MP.RecvTimeout(10);
  for (int i = 0; i < 8; ++i) {
    if (MP.Recv(&msgid, &value, coreId) < 0) {
      break;
    }
    Serial.print("[Main] Drained pending msg from ");
    Serial.print(coreName);
    Serial.print(": msgid=");
    Serial.print(msgid);
    Serial.print(" value=");
    Serial.println(value);
  }
  MP.RecvTimeout(3000);
}

bool configureSharedLayout(SharedMemoryLayout layout)
{
  if (g_allocator.isAllocated(g_shared_memory)) {
    if (!g_allocator.free(&g_shared_memory)) {
      Serial.println("[Main] shared memory fence check failed during layout switch");
    }
  }

  if (!g_allocator.alloc(layout, &g_shared_memory)) {
    Serial.print("[Main] shared memory allocation failed: layout=");
    Serial.println(layoutNameString[layout]);
    return false;
  }

  if (!sendAllSharedAddresses(g_shared_memory)) {
    Serial.println("[Main] failed to distribute shared addresses");
    return false;
  }

  g_active_layout = layout;
  applyModesForLayout(layout);
  Serial.print("[Main] shared layout=");
  Serial.println(layoutNameString[g_active_layout]);
  return true;
}



bool sendStartToAllSubcores()
{
  drainPendingFromCore(BMP280CORE_ID, "BMP280Core");
  drainPendingFromCore(BMI160CORE_ID, "BMI160Core");
  drainPendingFromCore(M_IMUCORE_ID, "MultiImuCore");

  /* Send START to BMP280Core with BMP mode */
  int ret = MP.Send(MSG_SENSOR_START, g_bmp_mode, BMP280CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(start->BMP) failed: ");
    Serial.println(ret);
    return false;
  }
  /* Wait for ACK from BMP280Core */
  if (!waitStateAckFromCore(BMP280CORE_ID, "BMP280Core")) {
    return false;
  }

  /* Send START to BMI160Core with BMI mode */
  ret = MP.Send(MSG_SENSOR_START, g_bmi_mode, BMI160CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(start->BMI) failed: ");
    Serial.println(ret);
    return false;
  }
  /* Wait for ACK from BMI160Core */
  if (!waitStateAckFromCore(BMI160CORE_ID, "BMI160Core")) {
    return false;
  }

  /* Send START to MultiImuCore with MIMU mode */
  ret = MP.Send(MSG_SENSOR_START, g_mimu_mode, M_IMUCORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(start->MIMU) failed: ");
    Serial.println(ret);
    return false;
  }
  /* Wait for ACK from MultiImuCore */
  if (!waitStateAckFromCore(M_IMUCORE_ID, "MultiImuCore")) {
    return false;
  }

  return true;
}

bool sendStopToAllSubcores()
{
  drainPendingFromCore(BMP280CORE_ID, "BMP280Core");
  drainPendingFromCore(BMI160CORE_ID, "BMI160Core");
  drainPendingFromCore(M_IMUCORE_ID, "MultiImuCore");

  int ret = MP.Send(MSG_SENSOR_STOP, (uint32_t)0, BMP280CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(stop->BMP) failed: ");
    Serial.println(ret);
    return false;
  }
  if (!waitStateAckFromCore(BMP280CORE_ID, "BMP280Core")) {
    return false;
  }

  ret = MP.Send(MSG_SENSOR_STOP, (uint32_t)0, BMI160CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(stop->BMI) failed: ");
    Serial.println(ret);
    return false;
  }
  if (!waitStateAckFromCore(BMI160CORE_ID, "BMI160Core")) {
    return false;
  }

  ret = MP.Send(MSG_SENSOR_STOP, (uint32_t)0, M_IMUCORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(stop->MIMU) failed: ");
    Serial.println(ret);
    return false;
  }
  if (!waitStateAckFromCore(M_IMUCORE_ID, "MultiImuCore")) {
    return false;
  }

  return true;
}

}  // namespace

static bool sendSharedAddressTo(int8_t msgid, int subid, void *addr)
{
  uint32_t phys = MP.Virt2Phys(addr);
  int ret = MP.Send(msgid, phys, subid);
  if (ret < 0) {
    Serial.print("[Main] failed to send shared address msg ");
    Serial.print(msgid);
    Serial.print(" to subcore ");
    Serial.print(subid);
    Serial.print(": ");
    Serial.println(ret);
    return false;
  }
  return true;
}

static bool sendAllSharedAddresses(const SharedMemoryRegion &region)
{
  if (!g_allocator.isAllocated(region)) {
    return false;
  }
  if (!sendSharedAddressTo(MSG_SET_SHARED_BMP, BMP280CORE_ID, region.bmp.data)) return false;
  if (!sendSharedAddressTo(MSG_SET_SHARED_BMI, BMI160CORE_ID, region.bmi.data)) return false;
  if (!sendSharedAddressTo(MSG_SET_SHARED_IMU, M_IMUCORE_ID,  region.mimu.data)) return false;
  return true;
}

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

  if (!configureSharedLayout(SHARED_LAYOUT_0)) {
    return;
  }

  Serial.println("[Main] setup done");
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {
  static MainState mainState = MAIN_STATE_READY;
  static uint32_t stateEnteredMs = millis();
  static bool firstRun = true;

  if (!g_allocator.isAllocated(g_shared_memory)) {
    delay(1000);
    return;
  }

  uint32_t now = millis();

  if (mainState == MAIN_STATE_RUN) {
    if ((uint32_t)(now - stateEnteredMs) >= MODE_SWITCH_INTERVAL_MS) {
      if (sendStopToAllSubcores()) {
        mainState = MAIN_STATE_READY;
        stateEnteredMs = now;
        Serial.println("[Main] mode=READY");
      }
    }
  }

  if (mainState == MAIN_STATE_READY) {
    if ((uint32_t)(now - stateEnteredMs) >= READY_HOLD_MS) {
      if (!firstRun) {
        SharedMemoryLayout next = nextLayout(g_active_layout);
        if (!configureSharedLayout(next)) {
          delay(200);
          return;
        }
      }
      if (sendStartToAllSubcores()) {
        firstRun = false;
        mainState = MAIN_STATE_RUN;
        stateEnteredMs = now;
        Serial.println("[Main] mode=RUN");
        // Give subcores a short time window right after START.
        delay(100);
      }
    }

    delay(50);
    return;
  }

  int8_t msgid;
  uint32_t seq = 0;
  int ret = 0;

  ret = MP.Send(MSG_REQ_BMP, (uint32_t)0, BMP280CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(BMP) failed: ");
    Serial.println(ret);
  }

  if (MP.Recv(&msgid, &seq, BMP280CORE_ID) >= 0 && msgid == MSG_RET_BMP) {
    if (seq == SENSOR_ERROR_OK && g_shared_memory.bmp.data) {
      float *bmpData = static_cast<float*>(g_shared_memory.bmp.data);
      printBMP(bmpData);
    }
  }

  ret = MP.Send(MSG_REQ_BMI, (uint32_t)0, BMI160CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(BMI) failed: ");
    Serial.println(ret);
  }

  if (MP.Recv(&msgid, &seq, BMI160CORE_ID) >= 0 && msgid == MSG_RET_BMI) {
    if (seq == SENSOR_ERROR_OK && g_shared_memory.bmi.data) {
      float *bmiData = static_cast<float*>(g_shared_memory.bmi.data);
      printBMI(bmiData);
    }
  }

  ret = MP.Send(MSG_REQ_MIMU, (uint32_t)0, M_IMUCORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(MIMU) failed: ");
    Serial.println(ret);
  }

  if (MP.Recv(&msgid, &seq, M_IMUCORE_ID) >= 0 && msgid == MSG_RET_MIMU) {
    if (seq == SENSOR_ERROR_OK && g_shared_memory.mimu.data) {
      float *mimuData = static_cast<float*>(g_shared_memory.mimu.data);
      printMIMU(mimuData);
    }
  }

  delay(1000);
}
