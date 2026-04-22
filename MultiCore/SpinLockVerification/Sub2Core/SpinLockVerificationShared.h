/*
 *  SpinLockVerificationShared.h - Shared definitions for SpinLockVerification multi-core sample.
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

#ifndef SPIN_LOCK_VERIFICATION_SHARED_H
#define SPIN_LOCK_VERIFICATION_SHARED_H

#include <stdint.h>

#define SUB1_CORE_ID 1
#define SUB2_CORE_ID 2
#define SUB3_CORE_ID 3
#define SUB4_CORE_ID 4
#define SUB5_CORE_ID 5

#define MSG_SET_LOCK   10
#define MSG_SET_REPORT 11
#define MSG_START      20
#define MSG_ACK        21

#define OWNER_MAIN 99u
#define OWNER_SUB1 1u
#define OWNER_SUB2 2u
#define OWNER_SUB3 3u
#define OWNER_SUB4 4u
#define OWNER_SUB5 5u

struct SpinLockDebugReport {
  volatile uint32_t owner;
  volatile uint32_t collisions;
  volatile uint32_t entries_main;
  volatile uint32_t entries_sub1;
  volatile uint32_t entries_sub2;
  volatile uint32_t entries_sub3;
  volatile uint32_t entries_sub4;
  volatile uint32_t entries_sub5;
  volatile uint32_t timeouts_main;
  volatile uint32_t timeouts_sub1;
  volatile uint32_t timeouts_sub2;
  volatile uint32_t timeouts_sub3;
  volatile uint32_t timeouts_sub4;
  volatile uint32_t timeouts_sub5;
};

#endif
