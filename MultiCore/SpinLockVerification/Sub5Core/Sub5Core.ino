/*
 *  Sub5Core.ino - SubCore 5 sketch for SpinLockVerification multi-core sample.
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

#if (SUBCORE != 5)
#error "Core selection is wrong!!"
#endif

#include <Arduino.h>
#include <MP.h>
#include "MultiCoreSpinLock.h"
#include "SpinLockVerificationShared.h"

constexpr uint32_t LOCK_SPIN_LIMIT = 120;
constexpr uint32_t HOLD_MS = 32;
constexpr uint32_t OUTER_DELAY_MS = 0;

static MultiCoreSpinLock *g_lock = NULL;
static SpinLockDebugReport *g_report = NULL;
static bool g_running = false;

static void processMessage(int8_t msgid, uint32_t value)
{
  if (msgid == MSG_SET_LOCK) {
    g_lock = reinterpret_cast<MultiCoreSpinLock *>(value);
  } else if (msgid == MSG_SET_REPORT) {
    g_report = reinterpret_cast<SpinLockDebugReport *>(value);
  } else if (msgid == MSG_START) {
    g_running = true;
    MP.Send(MSG_ACK, SUB5_CORE_ID);
  }
}

void setup()
{
  MP.begin();
  MP.RecvTimeout(5);
}

void loop()
{
  int8_t msgid = 0;
  uint32_t value = 0;
  if (MP.Recv(&msgid, &value, 0) >= 0) {
    processMessage(msgid, value);
  }

  if (!g_running || !g_lock || !g_report) {
    delay(1);
    return;
  }

  bool locked = MultiCoreSpin::acquire(g_lock, LOCK_SPIN_LIMIT);
  if (locked) {
    if (g_report->owner != 0u) {
      g_report->collisions++;
    }
    g_report->owner = OWNER_SUB5;
    g_report->entries_sub5++;
    delay(HOLD_MS);
    g_report->owner = 0u;
    MultiCoreSpin::release(g_lock);
  } else {
    g_report->timeouts_sub5++;
  }

  delay(OUTER_DELAY_MS);
}
