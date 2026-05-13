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

#include "ComparisonShared.h"
#include "SharedMemoryConfig.h"

static SharedMemoryRegion g_shared_memory = makeInitialSharedMemoryRegion();
static MultiCoreSpinLock *g_i2c_lock = NULL;
static SharedMemoryAllocator theAllocator;

static bool sendAllSharedAddresses(const SharedMemoryRegion &region);

namespace {

void printBMI(const BMIQuaternion &data) {
  Serial.print("[BMI ]");
  Serial.print("Q0="); Serial.print(data.q0, 7); Serial.print(" ");
  Serial.print("Q1="); Serial.print(data.q1, 7); Serial.print(" ");
  Serial.print("Q2="); Serial.print(data.q2, 7); Serial.print(" ");
  Serial.print("Q3="); Serial.print(data.q3, 7); Serial.print(" ");
  Serial.print("TS="); Serial.println(data.timestamp, 6);
}

void printMIMU(const MIMUQuaternion &data) {
  Serial.print("[MIMU]");
  Serial.print("Q0="); Serial.print(data.q0, 7); Serial.print(" ");
  Serial.print("Q1="); Serial.print(data.q1, 7); Serial.print(" ");
  Serial.print("Q2="); Serial.print(data.q2, 7); Serial.print(" ");
  Serial.print("Q3="); Serial.print(data.q3, 7); Serial.print(" ");
  Serial.print("TS="); Serial.print(data.timestamp, 6); Serial.print(" ");
  Serial.print("TEMP="); Serial.println(data.temperature, 2);
}

template<typename T>
bool readLatestLocked(InterCoreRingBuffer<T, RINGBUFFER_CAPACITY> *ring, T *latest, const char *label) {
  if (!ring || !latest) {
    Serial.print("[readLatestLocked] "); Serial.print(label);
    Serial.println(" NULL check failed");
    return false;
  }
  // InterCoreRingBuffer is lock-free (atomic ops). No spinlock needed here.
  bool hasData = false;
  T temp;
  while (ring->read(temp)) { *latest = temp; hasData = true; }
  return hasData;
}

bool configureSharedLayout() {
  if (theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) {
    reinterpret_cast<InterCoreRingBuffer<BMIQuaternion,  RINGBUFFER_CAPACITY> *>(g_shared_memory.bmi.data)->~InterCoreRingBuffer();
    reinterpret_cast<InterCoreRingBuffer<MIMUQuaternion, RINGBUFFER_CAPACITY> *>(g_shared_memory.mimu.data)->~InterCoreRingBuffer();
    if (!theAllocator.free(regionAreas(&g_shared_memory), SHARED_AREA_COUNT))
      Serial.println("[Main] shared memory fence check failed");
  }
  if (!theAllocator.alloc(sharedLayouts.areas, SHARED_AREA_COUNT, regionAreas(&g_shared_memory))) {
    Serial.println("[Main] shared memory allocation failed");
    return false;
  }
  g_i2c_lock = static_cast<MultiCoreSpinLock *>(g_shared_memory.spinlock.data);
  if (!g_i2c_lock) { Serial.println("[Main] spinlock null"); return false; }
  MultiCoreSpin::init(g_i2c_lock);
  new (g_shared_memory.bmi.data)  InterCoreRingBuffer<BMIQuaternion,  RINGBUFFER_CAPACITY>();
  new (g_shared_memory.mimu.data) InterCoreRingBuffer<MIMUQuaternion, RINGBUFFER_CAPACITY>();
  Serial.print("[Main] BMI RingBuffer at: ");  Serial.println((unsigned long)g_shared_memory.bmi.data);
  Serial.print("[Main] MIMU RingBuffer at: "); Serial.println((unsigned long)g_shared_memory.mimu.data);
  if (!sendAllSharedAddresses(g_shared_memory)) {
    Serial.println("[Main] failed to distribute shared addresses");
    return false;
  }
  Serial.println("[Main] shared layout configured");
  return true;
}

bool sendStartToAllSubcores() {
  Serial.println("[Main] about to send START to BMI...");
  int ret = MP.Send(MSG_SENSOR_START, (uint32_t)0, BMI160CORE_ID);
  if (ret < 0) { Serial.print("[Main] MP.Send(start->BMI) failed: "); Serial.println(ret); return false; }
  Serial.println("[Main] START sent to BMI, waiting...");
  delay(100);
  ret = MP.Send(MSG_SENSOR_START, (uint32_t)0, M_IMUCORE_ID);
  if (ret < 0) { Serial.print("[Main] MP.Send(start->MIMU) failed: "); Serial.println(ret); return false; }
  return true;
}

bool waitForSubcoreReady(int subcoreId, const char *name) {
  while (1) {
    MP.RecvTimeout(300);
    int8_t msgid = -1;
    uint32_t payload = 0;
    int ret = MP.Recv(&msgid, &payload, subcoreId);
    if (ret < 0) {
      continue;
    }
    if (msgid == MSG_SENSOR_READY) {
      Serial.print("[Main] READY from ");
      Serial.println(name);
      return true;
    }
    Serial.print("[Main] ignore msg from ");
    Serial.print(name);
    Serial.print(": ");
    Serial.println(msgid);
  }
}

bool sendStopToAllSubcores() {
  MP.Send(MSG_SENSOR_STOP, (uint32_t)0, BMI160CORE_ID);
  MP.Send(MSG_SENSOR_STOP, (uint32_t)0, M_IMUCORE_ID);
  return true;
}

}  // namespace

static bool sendSharedAddressTo(int8_t msgid, int subid, void *addr) {
  uint32_t phys = MP.Virt2Phys(addr);
  int ret = MP.Send(msgid, phys, subid);
  if (ret < 0) {
    Serial.print("[Main] send shared failed msg="); Serial.print(msgid);
    Serial.print(" sub="); Serial.println(subid);
    return false;
  }
  return true;
}

static bool sendAllSharedAddresses(const SharedMemoryRegion &region) {
  if (!theAllocator.isAllocated(regionAreas(&g_shared_memory), SHARED_AREA_COUNT)) return false;
  if (!sendSharedAddressTo(MSG_SET_SHARED_BMI, BMI160CORE_ID, region.bmi.data))  return false;
  if (!sendSharedAddressTo(MSG_SET_SHARED_IMU, M_IMUCORE_ID,  region.mimu.data)) return false;
  if (!sendSharedAddressTo(MSG_SET_LOCK, BMI160CORE_ID, g_i2c_lock)) return false;
  if (!sendSharedAddressTo(MSG_SET_LOCK, M_IMUCORE_ID,  g_i2c_lock)) return false;
  return true;
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[Main] setup begin");

  MP.begin(BMI160CORE_ID);

  if (!waitForSubcoreReady(BMI160CORE_ID, "BMI")) {
    while (1) delay(100);
  }

  MP.begin(M_IMUCORE_ID);

  if (!waitForSubcoreReady(M_IMUCORE_ID, "MIMU")) {
    while (1) delay(100);
  }

  MP.RecvTimeout(3000);

  Serial.println("[Main] about to configure shared layout");

  if (!configureSharedLayout()) {
    Serial.println("[Main] failed to configure shared layout");
    while (1) delay(100);
  }

  Serial.println("[Main] configured, sending start messages");

  if (!sendStartToAllSubcores()) {
    Serial.println("[Main] failed to start subcores");
    while (1) delay(100);
  }

  Serial.println("[Main] mode=RUN");
  Serial.println("[Main] setup done");
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {
  BMIQuaternion bmiLatest;
  MIMUQuaternion mimuLatest;

  auto bmiRing = static_cast<InterCoreRingBuffer<BMIQuaternion, RINGBUFFER_CAPACITY> *>(g_shared_memory.bmi.data);
  if (readLatestLocked(bmiRing, &bmiLatest, "BMI")) {
    printBMI(bmiLatest);
  }

  auto mimuRing = static_cast<InterCoreRingBuffer<MIMUQuaternion, RINGBUFFER_CAPACITY> *>(g_shared_memory.mimu.data);
  if (readLatestLocked(mimuRing, &mimuLatest, "MIMU")) {
    printMIMU(mimuLatest);
  }
}

