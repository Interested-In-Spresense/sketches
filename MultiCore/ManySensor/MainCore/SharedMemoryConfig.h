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

#include <SharedMemoryAllocator.h>
#include <MultiCoreSpinLock.h>

/* --- Sensor size-per-entry presets (bytes) --- */
/* BMP280: temp + pressure + altitude = FULL (3 floats), pressure only = PRESS (1 float) */
#define SHARED_BMP_BYTES_PRESS  (1 * sizeof(float))
#define SHARED_BMP_BYTES_FULL   (3 * sizeof(float))

/* BMI160: accel x/y/z = ACC (3 floats), gyro x/y/z = GYRO (3 floats), accel+gyro = FULL (6 floats) */
#define SHARED_BMI_BYTES_ACC    (3 * sizeof(float))
#define SHARED_BMI_BYTES_GYRO   (3 * sizeof(float))
#define SHARED_BMI_BYTES_FULL   (6 * sizeof(float))

/* MIMU: accel+gyro = AG (6 floats), accel+gyro+extended = FULL (8 floats) */
#define SHARED_MIMU_BYTES_DATA  (6 * sizeof(float))
#define SHARED_MIMU_BYTES_FULL  (8 * sizeof(float))

/* Backward compatibility for legacy typo-based macro names. */
#define SHARD_BMP_BYTES_PRESS   SHARED_BMP_BYTES_PRESS
#define SHARD_BMP_BYTES_FULL    SHARED_BMP_BYTES_FULL
#define SHARD_BMI_BYTES_ACC     SHARED_BMI_BYTES_ACC
#define SHARD_BMI_BYTES_GYRO    SHARED_BMI_BYTES_GYRO
#define SHARD_BMI_BYTES_FULL    SHARED_BMI_BYTES_FULL
#define SHARD_MIMU_BYTES_DATA   SHARED_MIMU_BYTES_DATA
#define SHARD_MIMU_BYTES_FULL   SHARED_MIMU_BYTES_FULL

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
  /* SHARED_LAYOUT_0: current sensor payload sizes */
  { { { sizeof(MultiCoreSpinLock), 1 },
    { SHARED_BMP_BYTES_FULL, 1 },
    { SHARED_BMI_BYTES_ACC, 1 },
    { SHARED_MIMU_BYTES_DATA, 1 } } },
  /* SHARED_LAYOUT_1: BMP pressure only, BMI accel+gyro, MIMU accel+gyro */
  { { { sizeof(MultiCoreSpinLock), 1 },
    { SHARED_BMP_BYTES_PRESS, 16 },
    { SHARED_BMI_BYTES_FULL, 16 },
    { SHARED_MIMU_BYTES_DATA, 64 } } },
  /* SHARED_LAYOUT_2: BMP full, BMI full, MIMU extended */
  { { { sizeof(MultiCoreSpinLock), 1 },
    { SHARED_BMP_BYTES_FULL, 16 },
    { SHARED_BMI_BYTES_FULL, 16 },
    { SHARED_MIMU_BYTES_FULL, 64 } } },
};

/* Debug print only: maps SharedMemoryLayout enum values to readable names. */
constexpr const char *layoutNameString[SHARED_LAYOUT_COUNT] = {
  "layout0",
  "layout1",
  "layout2",
};

#endif