/*
 *  MultiCoreSpinLock.h - Multi-core spin lock library for Spresense.
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

#ifndef MULTI_CORE_SPINLOCK_H
#define MULTI_CORE_SPINLOCK_H

#include <stdint.h>

struct MultiCoreSpinLock {
  volatile uint32_t state;
};

namespace MultiCoreSpin {

static inline void init(MultiCoreSpinLock *lock)
{
  if (lock) {
    __atomic_store_n(&lock->state, 0u, __ATOMIC_RELEASE);
  }
}

static inline bool tryAcquire(MultiCoreSpinLock *lock)
{
  if (!lock) {
    return true;
  }
  uint32_t expected = 0u;
  return __atomic_compare_exchange_n(&lock->state, &expected, 1u, false,
                                     __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

static inline bool acquire(MultiCoreSpinLock *lock, uint32_t spinLimit)
{
  if (!lock) {
    return true;
  }
  for (uint32_t i = 0; i < spinLimit; ++i) {
    if (tryAcquire(lock)) {
      return true;
    }
    delayMicroseconds(10);
  }
  return false;
}

static inline void release(MultiCoreSpinLock *lock)
{
  if (lock) {
    __atomic_store_n(&lock->state, 0u, __ATOMIC_RELEASE);
  }
}

}

#endif
