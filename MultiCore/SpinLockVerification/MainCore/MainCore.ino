/*
 *  MainCore.ino - Main core sketch for SpinLockVerification multi-core sample.
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
#include "MultiCoreSpinLock.h"
#include "SpinLockVerificationShared.h"

constexpr uint32_t LOCK_SPIN_LIMIT = 120;
constexpr uint32_t MAIN_HOLD_MS = 12;
constexpr uint32_t MAIN_OUTER_DELAY_MS = 0;

static MultiCoreSpinLock *g_lock = NULL;
static SpinLockDebugReport *g_report = NULL;

static void *allocSharedWithRetry(size_t bytes)
{
  for (int i = 0; i < 20; ++i) {
    void *ptr = MP.AllocSharedMemory(bytes);
    if (ptr) {
      return ptr;
    }
    delay(20);
  }
  return NULL;
}

static bool sendAddress(int8_t msgid, int subid, void *addr)
{
  uint32_t phys = MP.Virt2Phys(addr);
  int ret = MP.Send(msgid, phys, subid);
  return ret >= 0;
}

static bool waitAck(int subid)
{
  int8_t msgid = 0;
  uint32_t value = 0;
  int ret = MP.Recv(&msgid, &value, subid);
  return (ret >= 0 && msgid == MSG_ACK);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
  }

  MP.begin(SUB1_CORE_ID);
  MP.begin(SUB2_CORE_ID);
  MP.begin(SUB3_CORE_ID);
  MP.begin(SUB4_CORE_ID);
  MP.begin(SUB5_CORE_ID);
  MP.RecvTimeout(3000);

  g_lock = reinterpret_cast<MultiCoreSpinLock *>(allocSharedWithRetry(sizeof(MultiCoreSpinLock)));
  g_report = reinterpret_cast<SpinLockDebugReport *>(allocSharedWithRetry(sizeof(SpinLockDebugReport)));
  if (!g_lock || !g_report) {
    if (!g_lock) {
      Serial.println("[Main] shared memory allocation failed: lock");
    }
    if (!g_report) {
      Serial.println("[Main] shared memory allocation failed: report");
    }
    return;
  }

  MultiCoreSpin::init(g_lock);
  memset((void *)g_report, 0, sizeof(SpinLockDebugReport));

  if (!sendAddress(MSG_SET_LOCK, SUB1_CORE_ID, g_lock) ||
      !sendAddress(MSG_SET_REPORT, SUB1_CORE_ID, g_report) ||
      !sendAddress(MSG_SET_LOCK, SUB2_CORE_ID, g_lock) ||
      !sendAddress(MSG_SET_REPORT, SUB2_CORE_ID, g_report) ||
      !sendAddress(MSG_SET_LOCK, SUB3_CORE_ID, g_lock) ||
      !sendAddress(MSG_SET_REPORT, SUB3_CORE_ID, g_report) ||
      !sendAddress(MSG_SET_LOCK, SUB4_CORE_ID, g_lock) ||
      !sendAddress(MSG_SET_REPORT, SUB4_CORE_ID, g_report) ||
      !sendAddress(MSG_SET_LOCK, SUB5_CORE_ID, g_lock) ||
      !sendAddress(MSG_SET_REPORT, SUB5_CORE_ID, g_report)) {
    Serial.println("[Main] failed to send shared addresses");
    return;
  }

  if (MP.Send(MSG_START, (uint32_t)0, SUB1_CORE_ID) < 0 ||
      MP.Send(MSG_START, (uint32_t)0, SUB2_CORE_ID) < 0 ||
      MP.Send(MSG_START, (uint32_t)0, SUB3_CORE_ID) < 0 ||
      MP.Send(MSG_START, (uint32_t)0, SUB4_CORE_ID) < 0 ||
      MP.Send(MSG_START, (uint32_t)0, SUB5_CORE_ID) < 0) {
    Serial.println("[Main] failed to send start");
    return;
  }

  if (!waitAck(SUB1_CORE_ID) ||
      !waitAck(SUB2_CORE_ID) ||
      !waitAck(SUB3_CORE_ID) ||
      !waitAck(SUB4_CORE_ID) ||
      !waitAck(SUB5_CORE_ID)) {
    Serial.println("[Main] start ack timeout");
    return;
  }

  Serial.println("[Main] SpinLockVerification started");
}

void loop()
{
  if (!g_lock || !g_report) {
    delay(500);
    return;
  }

  bool locked = MultiCoreSpin::acquire(g_lock, LOCK_SPIN_LIMIT);
  if (locked) {
    if (g_report->owner != 0u) {
      g_report->collisions++;
    }
    g_report->owner = OWNER_MAIN;
    g_report->entries_main++;
    delay(MAIN_HOLD_MS);
    g_report->owner = 0u;
    MultiCoreSpin::release(g_lock);
  } else {
    g_report->timeouts_main++;
  }

  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - lastPrint) >= 1000u) {
    lastPrint = now;
    Serial.print("[Main] collisions=");
    Serial.print(g_report->collisions);
    Serial.print(" owner=");
    Serial.print(g_report->owner);
    Serial.print(" entries(main/sub1/sub2/sub3/sub4/sub5)=");
    Serial.print(g_report->entries_main);
    Serial.print("/");
    Serial.print(g_report->entries_sub1);
    Serial.print("/");
    Serial.print(g_report->entries_sub2);
    Serial.print("/");
    Serial.print(g_report->entries_sub3);
    Serial.print("/");
    Serial.print(g_report->entries_sub4);
    Serial.print("/");
    Serial.print(g_report->entries_sub5);
    Serial.print(" timeouts(main/sub1/sub2/sub3/sub4/sub5)=");
    Serial.print(g_report->timeouts_main);
    Serial.print("/");
    Serial.print(g_report->timeouts_sub1);
    Serial.print("/");
    Serial.print(g_report->timeouts_sub2);
    Serial.print("/");
    Serial.print(g_report->timeouts_sub3);
    Serial.print("/");
    Serial.print(g_report->timeouts_sub4);
    Serial.print("/");
    Serial.println(g_report->timeouts_sub5);
  }

  delay(MAIN_OUTER_DELAY_MS);
}
