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

#include <stddef.h>
#include <stdint.h>

/* --- Sensor size-per-entry presets (bytes) --- */
/* BMP280: temp + pressure + altitude = FULL (3 floats), pressure only = PRESS (1 float) */
#define SHARD_BMP_BYTES_PRESS   (1 * sizeof(float))
#define SHARD_BMP_BYTES_FULL    (3 * sizeof(float))

/* BMI160: accel x/y/z = ACC (3 floats), gyro x/y/z = GYRO (3 floats), accel+gyro = FULL (6 floats) */
#define SHARD_BMI_BYTES_ACC     (3 * sizeof(float))
#define SHARD_BMI_BYTES_GYRO    (3 * sizeof(float))
#define SHARD_BMI_BYTES_FULL    (6 * sizeof(float))

/* MIMU: accel+gyro = AG (6 floats), accel+gyro+extended = FULL (8 floats) */
#define SHARD_MIMU_BYTES_DATA   (6 * sizeof(float))
#define SHARD_MIMU_BYTES_FULL   (8 * sizeof(float))

enum SharedMemoryLayout {
  SHARED_LAYOUT_0 = 0,
  SHARED_LAYOUT_1,
  SHARED_LAYOUT_2,
  SHARED_LAYOUT_COUNT,
};

constexpr size_t SHARED_AREA_COUNT = 3;

struct SharedMemoryArea {
  void*     data;                 /* センサーデータ配列へのポインタ（フェンス後の実データ部） */
  size_t    bytes;                /* 1サンプルあたりのサイズ（バイト数） */
  size_t    count;                /* サンプル数（データ行数） */
  uint32_t* fence;                /* [内部用] フェンス付きメモリブロックの先頭（uint32_t*） */
  size_t    total_words;          /* [内部用] 確保メモリ全体（uint32_t単位にaligned）：dataWords + fence 2 words */
};

struct SharedMemoryRegion {
  SharedMemoryArea bmp;
  SharedMemoryArea bmi;
  SharedMemoryArea mimu;
};

inline SharedMemoryArea *regionAreas(SharedMemoryRegion *region)
{
  return &region->bmp;
}

inline const SharedMemoryArea *regionAreas(const SharedMemoryRegion *region)
{
  return &region->bmp;
}

struct SharedLayoutAreaConfig {
  size_t size;
  size_t count;
};

struct SharedLayoutMap {
  SharedLayoutAreaConfig areas[SHARED_AREA_COUNT];
};

constexpr SharedLayoutMap sharedLayouts[SHARED_LAYOUT_COUNT] = {
  /* SHARED_LAYOUT_0: current sensor payload sizes */
  { { { SHARD_BMP_BYTES_FULL, 1 },
      { SHARD_BMI_BYTES_ACC, 1 },
      { SHARD_MIMU_BYTES_DATA, 1 } } },
  /* SHARED_LAYOUT_1: BMP pressure only, BMI accel+gyro, MIMU accel+gyro */
  { { { SHARD_BMP_BYTES_PRESS, 16 },
      { SHARD_BMI_BYTES_FULL, 16 },
      { SHARD_MIMU_BYTES_DATA, 64 } } },
  /* SHARED_LAYOUT_2: BMP full, BMI full, MIMU extended */
  { { { SHARD_BMP_BYTES_FULL, 16 },
      { SHARD_BMI_BYTES_FULL, 16 },
      { SHARD_MIMU_BYTES_FULL, 64 } } },
};

/* Debug print only: maps SharedMemoryLayout enum values to readable names. */
constexpr const char *layoutNameString[SHARED_LAYOUT_COUNT] = {
  "layout0",
  "layout1",
  "layout2",
};

inline void applyLayoutMap(SharedMemoryArea *areas, size_t areaCount, const SharedLayoutMap &map)
{
  for (size_t index = 0; index < areaCount; ++index) {
    areas[index].bytes = map.areas[index].size;
    areas[index].count = map.areas[index].count;
  }
}

#endif