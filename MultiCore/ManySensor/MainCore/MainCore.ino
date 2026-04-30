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
#include "SharedMemoryConfig.h"
#include <MultiCoreSpinLock.h>
#include <SharedMemoryAllocator.h>

static SharedMemoryRegion g_shared_memory = {
  { NULL, sizeof(MultiCoreSpinLock), 1, NULL, 0 },
  { NULL, BMP280_DATA_LEN * sizeof(float), 0, NULL, 0 },
  { NULL, BMI160_DATA_LEN * sizeof(float), 0, NULL, 0 },
  { NULL, MULTIMU_DATA_LEN * sizeof(float), 0, NULL, 0 },
};

constexpr uint32_t MODE_SWITCH_INTERVAL_MS = 10000;
constexpr uint32_t READY_HOLD_MS = 300;
static uint32_t g_bmp_mode = BMP280_MODE_FULL;
static uint32_t g_bmi_mode = BMI160_MODE_ACC;
static uint32_t g_mimu_mode = MIMU_MODE_DATA;
static SharedMemoryLayout g_active_layout = SHARED_LAYOUT_2;
static MultiCoreSpinLock *g_i2c_lock = NULL;

static SharedMemoryAllocator theAllocator;
static bool sendAllSharedAddresses(const SharedMemoryRegion &region);

static bool waitSubcoreReady(int subid, const char *name)
{
  int8_t msgid = 0;
  uint32_t value = 0;
  int ret = MP.Recv(&msgid, &value, subid);
  if (ret < 0) {
    Serial.print("[Main] READY timeout from ");
    Serial.println(name);
    return false;
  }
  if (msgid != MSG_SUBCORE_READY) {
    Serial.print("[Main] unexpected startup msg from ");
    Serial.print(name);
    Serial.print(": msgid=");
    Serial.println(msgid);
    return false;
  }
  return true;
}

enum MainState {
  MAIN_STATE_RUN = 0,
  MAIN_STATE_READY,
};

namespace {

void printLayoutAreaInfo(const char *name, const SharedLayoutArea &area)
{
  Serial.print("[Main]   ");
  Serial.print(name);
  Serial.print(": size=");
  Serial.print((unsigned long)area.size);
  Serial.print(" count=");
  Serial.print((unsigned long)area.count);
  Serial.print(" total=");
  Serial.println((unsigned long)(area.size * area.count));
}

void printLayoutRequest(SharedMemoryLayout layout)
{
  Serial.print("[Main] configure request: ");
  Serial.println(layoutNameString[layout]);
  printLayoutAreaInfo("spinlock", sharedLayouts[layout].areas[0]);
  printLayoutAreaInfo("bmp", sharedLayouts[layout].areas[1]);
  printLayoutAreaInfo("bmi", sharedLayouts[layout].areas[2]);
  printLayoutAreaInfo("mimu", sharedLayouts[layout].areas[3]);
}

void printRegionState(const SharedMemoryRegion &region)
{
  Serial.print("[Main]   region spinlock data=");
  Serial.print((uintptr_t)region.spinlock.data, HEX);
  Serial.print(" total_bytes=");
  Serial.println((unsigned long)region.spinlock.total_bytes);
  Serial.print("[Main]   region bmp data=");
  Serial.print((uintptr_t)region.bmp.data, HEX);
  Serial.print(" total_bytes=");
  Serial.println((unsigned long)region.bmp.total_bytes);
  Serial.print("[Main]   region bmi data=");
  Serial.print((uintptr_t)region.bmi.data, HEX);
  Serial.print(" total_bytes=");
  Serial.println((unsigned long)region.bmi.total_bytes);
  Serial.print("[Main]   region mimu data=");
  Serial.print((uintptr_t)region.mimu.data, HEX);
  Serial.print(" total_bytes=");
  Serial.println((unsigned long)region.mimu.total_bytes);
}

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

bool configureSharedLayout(SharedMemoryLayout layout)
{
  printLayoutRequest(layout);
  Serial.print("[Main] allocator currently allocated=");
  Serial.println(theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT) ? "yes" : "no");

  if (theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
    Serial.println("[Main] freeing previous shared layout");
    if (!theAllocator.free(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
      Serial.println("[Main] shared memory fence check failed during layout switch");
    } else {
      Serial.println("[Main] previous shared layout freed");
    }
  }

  if (!theAllocator.alloc(sharedLayouts[layout].areas, SHARED_AREA_COUNT, regionAreas(&g_shared_memory))) {
    Serial.print("[Main] shared memory allocation failed: layout=");
    Serial.println(layoutNameString[layout]);
    printRegionState(g_shared_memory);
    return false;
  }

  g_i2c_lock = static_cast<MultiCoreSpinLock *>(g_shared_memory.spinlock.data);
  if (!g_i2c_lock) {
    Serial.println("[Main] spinlock allocation returned null");
    return false;
  }
  MultiCoreSpin::init(g_i2c_lock);
  Serial.print("[Main] i2c spinlock ready at 0x");
  Serial.println((uintptr_t)g_i2c_lock, HEX);

  Serial.println("[Main] shared memory allocation succeeded");
  printRegionState(g_shared_memory);

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
  int ret = MP.Send(MSG_SENSOR_START, g_bmp_mode, BMP280CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(start->BMP) failed: ");
    Serial.println(ret);
    return false;
  }

  ret = MP.Send(MSG_SENSOR_START, g_bmi_mode, BMI160CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(start->BMI) failed: ");
    Serial.println(ret);
    return false;
  }

  ret = MP.Send(MSG_SENSOR_START, g_mimu_mode, M_IMUCORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(start->MIMU) failed: ");
    Serial.println(ret);
    return false;
  }

  return true;
}

bool sendStopToAllSubcores()
{
  int ret = MP.Send(MSG_SENSOR_STOP, (uint32_t)0, BMP280CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(stop->BMP) failed: ");
    Serial.println(ret);
    return false;
  }

  ret = MP.Send(MSG_SENSOR_STOP, (uint32_t)0, BMI160CORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(stop->BMI) failed: ");
    Serial.println(ret);
    return false;
  }

  ret = MP.Send(MSG_SENSOR_STOP, (uint32_t)0, M_IMUCORE_ID);
  if (ret < 0) {
    Serial.print("[Main] MP.Send(stop->MIMU) failed: ");
    Serial.println(ret);
    return false;
  }

  return true;
}

}  // namespace

static bool sendSharedAddressTo(int8_t msgid, int subid, void *addr)
{
  uint32_t phys = MP.Virt2Phys(addr);
  Serial.print("[Main] send shared msg=");
  Serial.print(msgid);
  Serial.print(" sub=");
  Serial.print(subid);
  Serial.print(" virt=0x");
  Serial.print((uintptr_t)addr, HEX);
  Serial.print(" phys=0x");
  Serial.println(phys, HEX);
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
  if (!theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
    Serial.println("[Main] sendAllSharedAddresses: allocator reports not allocated");
    return false;
  }
  if (!g_i2c_lock) {
    Serial.println("[Main] sendAllSharedAddresses: i2c lock is null");
    return false;
  }
  if (!sendSharedAddressTo(MSG_SET_SHARED_BMP, BMP280CORE_ID, region.bmp.data)) return false;
  if (!sendSharedAddressTo(MSG_SET_SHARED_BMI, BMI160CORE_ID, region.bmi.data)) return false;
  if (!sendSharedAddressTo(MSG_SET_SHARED_IMU, M_IMUCORE_ID,  region.mimu.data)) return false;
  if (!sendSharedAddressTo(MSG_SET_LOCK, BMP280CORE_ID, g_i2c_lock)) return false;
  if (!sendSharedAddressTo(MSG_SET_LOCK, BMI160CORE_ID, g_i2c_lock)) return false;
  if (!sendSharedAddressTo(MSG_SET_LOCK, M_IMUCORE_ID,  g_i2c_lock)) return false;
  return true;
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("[Main] setup begin");

  MP.begin(BMI160CORE_ID);
  Serial.println("[Main] MP.begin BMI160 done");
  MP.begin(BMP280CORE_ID);
  Serial.println("[Main] MP.begin BMP280 done");
  MP.begin(M_IMUCORE_ID);
  Serial.println("[Main] MP.begin MIMU done");
  MP.RecvTimeout(3000);
  Serial.println("[Main] waiting subcore ready messages");

  if (!waitSubcoreReady(BMP280CORE_ID, "BMP280Core") ||
      !waitSubcoreReady(BMI160CORE_ID, "BMI160Core") ||
      !waitSubcoreReady(M_IMUCORE_ID, "MultiImuCore")) {
    Serial.println("[Main] setup aborted while waiting subcore ready");
    return;
  }

  Serial.println("[Main] all subcores ready");
  Serial.println("[Main] spinlock allocation is managed by shared layout");

  Serial.println("[Main] setup done");
  return;

FATAL_ERROR_SETUP:
  while (1) {
    usleep(1000 * 1000);
  }
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {
  static MainState mainState = MAIN_STATE_READY;
  static uint32_t stateEnteredMs = millis();
  static bool firstLayoutConfigured = false;
  SharedMemoryLayout next = SHARED_LAYOUT_0;

  uint32_t now = millis();

  if (!theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT) &&
      mainState == MAIN_STATE_RUN) {
    usleep(1000 * 1000);
    return;
  }

  if (mainState == MAIN_STATE_RUN) {
    if ((uint32_t)(now - stateEnteredMs) >= MODE_SWITCH_INTERVAL_MS) {
      if (sendStopToAllSubcores()) {
        if (theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
          if (!theAllocator.free(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
            Serial.println("[Main] FATAL: shared memory fence check failed on stop");
            goto FATAL_ERROR;
          }
        }
        mainState = MAIN_STATE_READY;
        stateEnteredMs = now;
        Serial.println("[Main] mode=READY");
        usleep(100 * 1000);
      } else {
        Serial.println("[Main] FATAL: failed to stop subcores");
        goto FATAL_ERROR;
      }
    }
  }

  if (mainState == MAIN_STATE_READY) {
    if ((uint32_t)(now - stateEnteredMs) >= READY_HOLD_MS) {
      next = firstLayoutConfigured ? nextLayout(g_active_layout) : SHARED_LAYOUT_2;
      if (!configureSharedLayout(next)) {
        return;
      }
      firstLayoutConfigured = true;
      if (sendStartToAllSubcores()) {
        mainState = MAIN_STATE_RUN;
        stateEnteredMs = now;
        Serial.println("[Main] mode=RUN");
        usleep(100 * 1000);
      } else {
        Serial.println("[Main] FATAL: failed to start subcores");
        goto FATAL_ERROR;
      }
    }
    return;
  }

  if (g_shared_memory.bmp.data) {
    float *bmpData = static_cast<float*>(g_shared_memory.bmp.data);
    printBMP(bmpData);
  }

  if (g_shared_memory.bmi.data) {
    float *bmiData = static_cast<float*>(g_shared_memory.bmi.data);
    printBMI(bmiData);
  }

  if (g_shared_memory.mimu.data) {
    float *mimuData = static_cast<float*>(g_shared_memory.mimu.data);
    printMIMU(mimuData);
  }
  return;

FATAL_ERROR:
  while (1) {
    usleep(1000 * 1000);
  }
}
