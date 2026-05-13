/*
 *  SharedMemoryConfig.h - Shared memory layout configuration for Comparison.
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

/*
 *  SharedMemoryConfig.h - Shared memory layout configuration for Comparison.
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

#ifndef SHARED_MEMORY_CONFIG_H
#define SHARED_MEMORY_CONFIG_H

#include <InterCoreRingBuffer.h>
#include <SharedMemoryAllocator.h>
#include <MultiCoreSpinLock.h>
#include "ComparisonShared.h"

#define SHARED_BMI_RINGBUF_BYTES  (sizeof(InterCoreRingBuffer<BMIQuaternion,  RINGBUFFER_CAPACITY>))
#define SHARED_MIMU_RINGBUF_BYTES (sizeof(InterCoreRingBuffer<MIMUQuaternion, RINGBUFFER_CAPACITY>))

constexpr size_t SHARED_AREA_COUNT = 3;

struct SharedMemoryRegion {
  SharedMemoryArea spinlock;
  SharedMemoryArea bmi;
  SharedMemoryArea mimu;
};

inline SharedMemoryRegion makeInitialSharedMemoryRegion()
{
  SharedMemoryRegion region = {
    { NULL, sizeof(MultiCoreSpinLock),  1, NULL, 0 },
    { NULL, SHARED_BMI_RINGBUF_BYTES,   1, NULL, 0 },
    { NULL, SHARED_MIMU_RINGBUF_BYTES,  1, NULL, 0 },
  };
  return region;
}

inline SharedMemoryArea *regionAreas(SharedMemoryRegion *region)
{
  return &region->spinlock;
}

inline const SharedMemoryArea *regionAreas(const SharedMemoryRegion *region)
{
  return &region->spinlock;
}

struct SharedLayoutMap {
  SharedLayoutArea areas[SHARED_AREA_COUNT];
};

const SharedLayoutMap sharedLayouts = {
  { { sizeof(MultiCoreSpinLock),  1 },
    { SHARED_BMI_RINGBUF_BYTES,   1 },
    { SHARED_MIMU_RINGBUF_BYTES,  1 } }
};

#endif