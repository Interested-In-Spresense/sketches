/*
 *  SharedMemoryConfig.h - Shared memory layout configuration for ManySensor.
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
 *  SharedMemoryConfig.h - Shared memory layout configuration for ManySensor.
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
#include "ManySensorShared.h"

/* Shared memory stores per-core ringbuffer instances (one each). */
#define SHARED_BMP_RINGBUF_BYTES_FULL  (sizeof(InterCoreRingBuffer<BMP280Full,  RINGBUFFER_CAPACITY>))
#define SHARED_BMP_RINGBUF_BYTES_PRESS (sizeof(InterCoreRingBuffer<BMP280Press, RINGBUFFER_CAPACITY>))
#define SHARED_BMI_RINGBUF_BYTES_ACC   (sizeof(InterCoreRingBuffer<BMI160Acc,   RINGBUFFER_CAPACITY>))
#define SHARED_BMI_RINGBUF_BYTES_IMU   (sizeof(InterCoreRingBuffer<BMI160Imu,   RINGBUFFER_CAPACITY>))
#define SHARED_MIMU_RINGBUF_BYTES_RAW  (sizeof(InterCoreRingBuffer<MIMURaw,     RINGBUFFER_CAPACITY>))
#define SHARED_MIMU_RINGBUF_BYTES_FULL (sizeof(InterCoreRingBuffer<MIMUFull,    RINGBUFFER_CAPACITY>))

enum SharedMemoryLayout {
  SHARED_LAYOUT_0 = 0,
  SHARED_LAYOUT_1,
  SHARED_LAYOUT_2,
  SHARED_LAYOUT_COUNT,
};

constexpr size_t SHARED_AREA_COUNT = 4;

struct SharedMemoryRegion {
  SharedMemoryArea spinlock;
  SharedMemoryArea bmp;
  SharedMemoryArea bmi;
  SharedMemoryArea mimu;
};

inline SharedMemoryRegion makeInitialSharedMemoryRegion()
{
  SharedMemoryRegion region = {
    { NULL, sizeof(MultiCoreSpinLock),         1, NULL, 0 },
    { NULL, SHARED_BMP_RINGBUF_BYTES_FULL,  1, NULL, 0 },
    { NULL, SHARED_BMI_RINGBUF_BYTES_ACC,   1, NULL, 0 },
    { NULL, SHARED_MIMU_RINGBUF_BYTES_RAW,  1, NULL, 0 },
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

constexpr SharedLayoutMap sharedLayouts[SHARED_LAYOUT_COUNT] = {
  /* SHARED_LAYOUT_0: BMP_FULL, BMI_ACC, MIMU_RAW */
  { { { sizeof(MultiCoreSpinLock), 1 },
    { SHARED_BMP_RINGBUF_BYTES_FULL,  1 },
    { SHARED_BMI_RINGBUF_BYTES_ACC,   1 },
    { SHARED_MIMU_RINGBUF_BYTES_RAW,  1 } } },
  /* SHARED_LAYOUT_1: BMP_PRESS, BMI_IMU, MIMU_RAW */
  { { { sizeof(MultiCoreSpinLock), 1 },
    { SHARED_BMP_RINGBUF_BYTES_PRESS, 1 },
    { SHARED_BMI_RINGBUF_BYTES_IMU,   1 },
    { SHARED_MIMU_RINGBUF_BYTES_RAW,  1 } } },
  /* SHARED_LAYOUT_2: BMP_FULL, BMI_IMU, MIMU_FULL */
  { { { sizeof(MultiCoreSpinLock), 1 },
    { SHARED_BMP_RINGBUF_BYTES_FULL,  1 },
    { SHARED_BMI_RINGBUF_BYTES_IMU,   1 },
    { SHARED_MIMU_RINGBUF_BYTES_FULL, 1 } } },
};

/* Debug print only: maps SharedMemoryLayout enum values to readable names. */
constexpr const char *layoutNameString[SHARED_LAYOUT_COUNT] = {
  "layout0",
  "layout1",
  "layout2",
};

#endif