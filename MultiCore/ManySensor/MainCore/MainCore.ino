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
#include <new>
#include <InterCoreRingBuffer.h>
#include <MultiCoreSpinLock.h>
#include <SharedMemoryAllocator.h>

#include "ManySensorShared.h"
#include "SharedMemoryConfig.h"

static SharedMemoryRegion g_shared_memory = makeInitialSharedMemoryRegion();

constexpr uint32_t MODE_SWITCH_INTERVAL_MS = 10000;
constexpr uint32_t READY_HOLD_MS = 300;
constexpr uint32_t LOCK_SPIN_LIMIT = 3000;
static uint32_t g_bmp_mode = BMP280_MODE_FULL;
static uint32_t g_bmi_mode = BMI160_MODE_ACC;
static uint32_t g_mimu_mode = MIMU_MODE_DATA;
static SharedMemoryLayout g_active_layout = SHARED_LAYOUT_2;
static MultiCoreSpinLock *g_i2c_lock = NULL;

#ifdef ENABLE_DIAG
constexpr uint32_t DIAG_PRINT_INTERVAL_MS = 2000;

struct SensorDiag {
  uint32_t okCount;
  uint32_t noDataCount;
  uint32_t lastOkMs;
};

static SensorDiag g_bmi_diag = {0, 0, 0};
static SensorDiag g_mimu_diag = {0, 0, 0};
static uint32_t g_diag_last_print_ms = 0;
#endif

static SharedMemoryAllocator theAllocator;
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

#ifdef ENABLE_DIAG
void recordBmiDiag(bool ok, uint32_t now)
{
  if (ok) {
    g_bmi_diag.okCount++;
    g_bmi_diag.lastOkMs = now;
  } else {
    g_bmi_diag.noDataCount++;
  }
}

void recordMimuDiag(bool ok, uint32_t now)
{
  if (ok) {
    g_mimu_diag.okCount++;
    g_mimu_diag.lastOkMs = now;
  } else {
    g_mimu_diag.noDataCount++;
  }
}

void printDiagIfDue(uint32_t now)
{
  if ((uint32_t)(now - g_diag_last_print_ms) < DIAG_PRINT_INTERVAL_MS) {
    return;
  }

  g_diag_last_print_ms = now;
  Serial.print("[DIAG] layout=");
  Serial.print(layoutNameString[g_active_layout]);
  Serial.print(" bmiMode=");
  Serial.print(g_bmi_mode);
  Serial.print(" bmiOk=");
  Serial.print(g_bmi_diag.okCount);
  Serial.print(" bmiMiss=");
  Serial.print(g_bmi_diag.noDataCount);
  Serial.print(" bmiLastOkAgoMs=");
  Serial.print((uint32_t)(now - g_bmi_diag.lastOkMs));
  Serial.print(" mimuMode=");
  Serial.print(g_mimu_mode);
  Serial.print(" mimuOk=");
  Serial.print(g_mimu_diag.okCount);
  Serial.print(" mimuMiss=");
  Serial.print(g_mimu_diag.noDataCount);
  Serial.print(" mimuLastOkAgoMs=");
  Serial.println((uint32_t)(now - g_mimu_diag.lastOkMs));
}
#else
void recordBmiDiag(bool ok, uint32_t now)
{
  (void)ok;
  (void)now;
}

void recordMimuDiag(bool ok, uint32_t now)
{
  (void)ok;
  (void)now;
}

void printDiagIfDue(uint32_t now)
{
  (void)now;
}
#endif

void printBMP(const BMP280Press &data)
{
  Serial.print("[BMP] pressure=");
  Serial.print(data.pressure);
  Serial.println("Pa");
}

void printBMP(const BMP280Full &data)
{
  Serial.print("[BMP] temp=");
  Serial.print(data.temperature);
  Serial.print("C pressure=");
  Serial.print(data.pressure);
  Serial.print("Pa altitude=");
  Serial.print(data.altitude);
  Serial.println("m");
}

void printBMI(const BMI160Acc &data)
{
  Serial.print("[BMI] acc[m/s^2]=");
  Serial.print(data.ax);
  Serial.print(", ");
  Serial.print(data.ay);
  Serial.print(", ");
  Serial.println(data.az);
}

void printBMI(const BMI160Gyro &data)
{
  Serial.print("[BMI] gyro[rad/s]=");
  Serial.print(data.gx, 4);
  Serial.print(", ");
  Serial.print(data.gy, 4);
  Serial.print(", ");
  Serial.println(data.gz, 4);
}

void printBMI(const BMI160Imu &data)
{
  Serial.print("[BMI] acc[m/s^2]=");
  Serial.print(data.ax);
  Serial.print(", ");
  Serial.print(data.ay);
  Serial.print(", ");
  Serial.print(data.az);
  Serial.print(" gyro[rad/s]=");
  Serial.print(data.gx, 4);
  Serial.print(", ");
  Serial.print(data.gy, 4);
  Serial.print(", ");
  Serial.println(data.gz, 4);
}

void printMIMU(const MIMURaw &data)
{
  Serial.print("[MIMU] acc[m/s^2]=");
  Serial.print(data.ax);
  Serial.print(", ");
  Serial.print(data.ay);
  Serial.print(", ");
  Serial.print(data.az);
  Serial.print(" gyro[rad/s]=");
  Serial.print(data.gx, 4);
  Serial.print(", ");
  Serial.print(data.gy, 4);
  Serial.print(", ");
  Serial.println(data.gz, 4);
}

void printMIMU(const MIMUFull &data)
{
  Serial.print("[MIMU] ts=");
  Serial.print(data.timestamp);
  Serial.print(" temp[C]=");
  Serial.print(data.temperature);
  Serial.print(" acc[m/s^2]=");
  Serial.print(data.ax);
  Serial.print(", ");
  Serial.print(data.ay);
  Serial.print(", ");
  Serial.print(data.az);
  Serial.print(" gyro[rad/s]=");
  Serial.print(data.gx, 4);
  Serial.print(", ");
  Serial.print(data.gy, 4);
  Serial.print(", ");
  Serial.println(data.gz, 4);
}

template<typename T>
bool readLatestLocked(InterCoreRingBuffer<T, RINGBUFFER_CAPACITY> *ring, T *latest)
{
  if (!ring || !latest || !g_i2c_lock) {
    return false;
  }
  if (!MultiCoreSpin::acquire(g_i2c_lock, LOCK_SPIN_LIMIT)) {
    return false;
  }

  bool hasData = false;
  T temp;
  while (ring->read(temp)) {
    *latest = temp;
    hasData = true;
  }

  MultiCoreSpin::release(g_i2c_lock);
  return hasData;
}

bool configureSharedLayout(SharedMemoryLayout layout)
{
  if (theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
    if (g_shared_memory.bmp.data) {
      if (g_bmp_mode == BMP280_MODE_PRESS) {
        reinterpret_cast<InterCoreRingBuffer<BMP280Press, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmp.data)->~InterCoreRingBuffer();
      } else {
        reinterpret_cast<InterCoreRingBuffer<BMP280Full, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmp.data)->~InterCoreRingBuffer();
      }
    }
    if (g_shared_memory.bmi.data) {
      if (g_bmi_mode == BMI160_MODE_ACC) {
        reinterpret_cast<InterCoreRingBuffer<BMI160Acc, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmi.data)->~InterCoreRingBuffer();
      } else {
        reinterpret_cast<InterCoreRingBuffer<BMI160Imu, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmi.data)->~InterCoreRingBuffer();
      }
    }
    if (g_shared_memory.mimu.data) {
      if (g_mimu_mode == MIMU_MODE_FULL) {
        reinterpret_cast<InterCoreRingBuffer<MIMUFull, RINGBUFFER_CAPACITY> *>(g_shared_memory.mimu.data)->~InterCoreRingBuffer();
      } else {
        reinterpret_cast<InterCoreRingBuffer<MIMURaw, RINGBUFFER_CAPACITY> *>(g_shared_memory.mimu.data)->~InterCoreRingBuffer();
      }
    }

    if (!theAllocator.free(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
      Serial.println("[Main] shared memory fence check failed during layout switch");
    }
  }

  if (!theAllocator.alloc(sharedLayouts[layout].areas, SHARED_AREA_COUNT, regionAreas(&g_shared_memory))) {
    Serial.print("[Main] shared memory allocation failed: ");
    Serial.println(layoutNameString[layout]);
    return false;
  }

  g_i2c_lock = static_cast<MultiCoreSpinLock *>(g_shared_memory.spinlock.data);
  if (!g_i2c_lock) {
    Serial.println("[Main] spinlock allocation returned null");
    return false;
  }
  MultiCoreSpin::init(g_i2c_lock);

  applyModesForLayout(layout);

  if (g_bmp_mode == BMP280_MODE_PRESS) {
    new (g_shared_memory.bmp.data) InterCoreRingBuffer<BMP280Press, RINGBUFFER_CAPACITY>();
  } else {
    new (g_shared_memory.bmp.data) InterCoreRingBuffer<BMP280Full, RINGBUFFER_CAPACITY>();
  }
  if (g_bmi_mode == BMI160_MODE_ACC) {
    new (g_shared_memory.bmi.data) InterCoreRingBuffer<BMI160Acc, RINGBUFFER_CAPACITY>();
  } else {
    new (g_shared_memory.bmi.data) InterCoreRingBuffer<BMI160Imu, RINGBUFFER_CAPACITY>();
  }
  if (g_mimu_mode == MIMU_MODE_FULL) {
    new (g_shared_memory.mimu.data) InterCoreRingBuffer<MIMUFull, RINGBUFFER_CAPACITY>();
  } else {
    new (g_shared_memory.mimu.data) InterCoreRingBuffer<MIMURaw, RINGBUFFER_CAPACITY>();
  }

  if (!sendAllSharedAddresses(g_shared_memory)) {
    Serial.println("[Main] failed to distribute shared addresses");
    return false;
  }

  g_active_layout = layout;
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
  int ret = MP.Send(msgid, phys, subid);
  if (ret < 0) {
    Serial.print("[Main] send shared failed msg=");
    Serial.print(msgid);
    Serial.print(" sub=");
    Serial.println(subid);
    return false;
  }
  return true;
}

static bool sendAllSharedAddresses(const SharedMemoryRegion &region)
{
  if (!theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
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
  MP.begin(BMP280CORE_ID);
  MP.begin(M_IMUCORE_ID);
  MP.RecvTimeout(3000);

  Serial.println("[Main] setup done");
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {
  static MainState mainState = MAIN_STATE_READY;
  static uint32_t stateEnteredMs = millis();

  uint32_t now = millis();

  if (mainState == MAIN_STATE_RUN) {
    if ((uint32_t)(now - stateEnteredMs) >= MODE_SWITCH_INTERVAL_MS) {
      if (sendStopToAllSubcores()) {
        mainState = MAIN_STATE_READY;
        stateEnteredMs = now;
        Serial.println("[Main] mode=READY");
        delay(100);
      } else {
        Serial.println("[Main] failed to stop subcores");
      }
    }
  }

  if (mainState == MAIN_STATE_READY) {
    if ((uint32_t)(now - stateEnteredMs) >= READY_HOLD_MS) {
      SharedMemoryLayout next = nextLayout(g_active_layout);
      if (!configureSharedLayout(next)) {
        delay(100);
        return;
      }

      if (sendStartToAllSubcores()) {
        mainState = MAIN_STATE_RUN;
        stateEnteredMs = now;
        Serial.println("[Main] mode=RUN");
        delay(100);
      } else {
        Serial.println("[Main] failed to start subcores");
      }
    }
    return;
  }

  if (g_shared_memory.bmp.data) {
    if (g_bmp_mode == BMP280_MODE_PRESS) {
      auto ring = static_cast<InterCoreRingBuffer<BMP280Press, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmp.data);
      BMP280Press latest;
      if (readLatestLocked(ring, &latest)) {
        printBMP(latest);
      }
    } else {
      auto ring = static_cast<InterCoreRingBuffer<BMP280Full, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmp.data);
      BMP280Full latest;
      if (readLatestLocked(ring, &latest)) {
        printBMP(latest);
      }
    }
  }

  if (g_shared_memory.bmi.data) {
    if (g_bmi_mode == BMI160_MODE_ACC) {
      auto ring = static_cast<InterCoreRingBuffer<BMI160Acc, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmi.data);
      BMI160Acc latest;
      bool hasData = readLatestLocked(ring, &latest);
      recordBmiDiag(hasData, now);
      if (hasData) {
        printBMI(latest);
      }
    } else if (g_bmi_mode == BMI160_MODE_GYRO) {
      auto ring = static_cast<InterCoreRingBuffer<BMI160Gyro, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmi.data);
      BMI160Gyro latest;
      bool hasData = readLatestLocked(ring, &latest);
      recordBmiDiag(hasData, now);
      if (hasData) {
        printBMI(latest);
      }
    } else {
      auto ring = static_cast<InterCoreRingBuffer<BMI160Imu, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmi.data);
      BMI160Imu latest;
      bool hasData = readLatestLocked(ring, &latest);
      recordBmiDiag(hasData, now);
      if (hasData) {
        printBMI(latest);
      }
    }
  }

  if (g_shared_memory.mimu.data) {
    if (g_mimu_mode == MIMU_MODE_FULL) {
      auto ring = static_cast<InterCoreRingBuffer<MIMUFull, RINGBUFFER_CAPACITY> *>(g_shared_memory.mimu.data);
      MIMUFull latest;
      bool hasData = readLatestLocked(ring, &latest);
      recordMimuDiag(hasData, now);
      if (hasData) {
        printMIMU(latest);
      }
    } else {
      auto ring = static_cast<InterCoreRingBuffer<MIMURaw, RINGBUFFER_CAPACITY> *>(g_shared_memory.mimu.data);
      MIMURaw latest;
      bool hasData = readLatestLocked(ring, &latest);
      recordMimuDiag(hasData, now);
      if (hasData) {
        printMIMU(latest);
      }
    }
  }

  printDiagIfDue(now);
}
