/*
 *  SharedMemoryAllocator.cpp - Shared memory allocator for ManySensor.
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

#include <Arduino.h>
#include <MP.h>

#include "SharedMemoryAllocator.h"

const uint32_t SHARD_MEMORY_FENCE_PATTERN = 0xDEADBEEF;

void ShardMemoryAllocator::resetArea(SharedMemoryArea *area)
{
  area->data = NULL;
  area->bytes = 0;
  area->count = 0;
  area->fence = NULL;
  area->total_words = 0;
}

void ShardMemoryAllocator::initFenceWords(uint32_t *rawWords, size_t totalWords)
{
  rawWords[0] = SHARD_MEMORY_FENCE_PATTERN;
  rawWords[totalWords - 1] = SHARD_MEMORY_FENCE_PATTERN;
}

bool ShardMemoryAllocator::allocSensorArea(size_t entrySizeBytes,
                                           size_t entryCount,
                                           uint32_t **rawWords,
                                           size_t *totalWords,
                                           void **data)
{
  const size_t dataBytes = entrySizeBytes * entryCount;
  const size_t dataWords = (dataBytes + sizeof(uint32_t) - 1) / sizeof(uint32_t);
  *totalWords = dataWords + 2;

  *rawWords = static_cast<uint32_t *>(MP.AllocSharedMemory(sizeof(uint32_t) * (*totalWords)));
  if (!*rawWords) {
    *totalWords = 0;
    *data = NULL;
    return false;
  }

  initFenceWords(*rawWords, *totalWords);
  *data = reinterpret_cast<void *>((*rawWords) + 1);
  return true;
}

bool ShardMemoryAllocator::alloc(SharedMemoryLayout layout, SharedMemoryRegion *region)
{
  SharedMemoryArea *areas = regionAreas(region);
  const size_t areaCount = SHARED_AREA_COUNT;

  if (layout >= SHARED_LAYOUT_COUNT || areaCount != SHARED_AREA_COUNT) {
    return false;
  }

  for (size_t index = 0; index < areaCount; ++index) {
    resetArea(&areas[index]);
  }

  applyLayoutMap(areas, areaCount, sharedLayouts[layout]);

  for (size_t index = 0; index < areaCount; ++index) {
    if (!allocSensorArea(areas[index].bytes,
                         areas[index].count,
                         &areas[index].fence,
                         &areas[index].total_words,
                         &areas[index].data)) {
      free(region);
      return false;
    }
  }

  if (!isAllocated(*region)) {
    free(region);
    return false;
  }

  clearData(areas, areaCount);
  return true;
}

bool ShardMemoryAllocator::free(SharedMemoryRegion *region)
{
  SharedMemoryArea *areas = regionAreas(region);
  const size_t areaCount = SHARED_AREA_COUNT;

  bool fenceOk = true;

  for (size_t index = 0; index < areaCount; ++index) {
    if (areas[index].fence) {
      fenceOk = fenceOk && areas[index].total_words >= 2 &&
                areas[index].fence[0] == SHARD_MEMORY_FENCE_PATTERN &&
                areas[index].fence[areas[index].total_words - 1] == SHARD_MEMORY_FENCE_PATTERN;
    }
  }

  if (!fenceOk) {
    Serial.println("[Main] shared memory fence broken before free");
  }

  for (size_t index = 0; index < areaCount; ++index) {
    if (areas[index].fence) {
      MP.FreeSharedMemory(areas[index].fence);
    }
    resetArea(&areas[index]);
  }

  return fenceOk;
}

void ShardMemoryAllocator::clearData(SharedMemoryArea *areas, size_t areaCount)
{
  for (size_t index = 0; index < areaCount; ++index) {
    if (areas[index].data) {
      memset(areas[index].data, 0,
             areas[index].bytes * areas[index].count);
    }
  }
}

bool ShardMemoryAllocator::isAllocated(const SharedMemoryRegion &region) const
{
  const SharedMemoryArea *areas = regionAreas(&region);
  const size_t areaCount = SHARED_AREA_COUNT;

  if (areaCount == 0) {
    return false;
  }

  for (size_t index = 0; index < areaCount; ++index) {
    if (!areas[index].data) {
      return false;
    }
  }
  return true;
}
