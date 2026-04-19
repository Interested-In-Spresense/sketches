/*
 *  SharedMemoryAllocator.h - Shared memory allocator for ManySensor.
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

#ifndef SHARD_MEMORY_ALLOCATOR_H
#define SHARD_MEMORY_ALLOCATOR_H

#include "SharedMemoryConfig.h"

class ShardMemoryAllocator {
public:
  bool alloc(SharedMemoryLayout layout, SharedMemoryRegion *region);
  bool free(SharedMemoryRegion *region);
  bool isAllocated(const SharedMemoryRegion &region) const;

private:
  static void resetArea(SharedMemoryArea *area);
  static void clearData(SharedMemoryArea *areas, size_t areaCount);
  static bool allocSensorArea(size_t entrySizeBytes, size_t entryCount,
                              uint32_t **rawWords, size_t *totalWords, void **data);
  static void initFenceWords(uint32_t *rawWords, size_t totalWords);
};

#endif
