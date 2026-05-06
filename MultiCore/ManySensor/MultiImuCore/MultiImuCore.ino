/*
 *  MultiImuCore.ino - MultiIMU sensor core for ManySensor.
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

#if (SUBCORE != 3)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <InterCoreRingBuffer.h>
#include <MultiCoreSpinLock.h>
#include "SpresenseIMU.h"
#include "ManySensorShared.h"

#define SAMPLINGRATE      (1920)
#define ADRANGE           (4)
#define GDRANGE           (500)
#define FIFO_DEPTH        (1)

static void *g_ring = NULL;
static MultiCoreSpinLock *g_i2c_lock = NULL;
static uint32_t g_state = SENSOR_STATE_READY;
static uint32_t g_mode = MIMU_MODE_DATA;

static void haltOnError(int32_t errCode)
{
  (void)errCode;
  while (1) {
    usleep(1000 * 1000);
  }
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  int ret = SpresenseIMU.begin();
  if (ret < 0) {
    haltOnError(SENSOR_ERR_MIMU_BEGIN_FAILED);
  }

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret) {
    haltOnError(SENSOR_ERR_MIMU_INIT_FAILED);
  }

  ret = SpresenseIMU.start();
  if (!ret) {
    haltOnError(SENSOR_ERR_MIMU_START_FAILED);
  }

  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  MP.begin();
  MP.RecvTimeout(10);
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {
  int8_t msgid;
  uint32_t payload;

  int ret = MP.Recv(&msgid, &payload, 0);
  if (ret >= 0) {
    switch (msgid) {
    case MSG_SET_SHARED_IMU:
      g_ring = reinterpret_cast<void *>(payload);
      return;
    case MSG_SET_LOCK:
      g_i2c_lock = reinterpret_cast<MultiCoreSpinLock *>(payload);
      return;
    case MSG_SENSOR_STOP:
      g_state = SENSOR_STATE_READY;
      return;
    case MSG_SENSOR_START:
      g_state = SENSOR_STATE_RUN;
      g_mode = (uint32_t)payload;
      return;
    default:
      haltOnError(SENSOR_ERR_MIMU_UNEXPECTED_MSG);
    }
  }

  if (g_state != SENSOR_STATE_RUN || !g_ring) {
    return;
  }

  pwbImuData d;
  if (!SpresenseIMU.get(d)) {
    return;
  }

  if (g_mode == MIMU_MODE_FULL) {
    MIMUFull sample = {
      (uint32_t)(d.data.timestamp / 19200000.0f),
      d.data.temp,
      d.data.ax,
      d.data.ay,
      d.data.az,
      d.data.gx,
      d.data.gy,
      d.data.gz
    };
    reinterpret_cast<InterCoreRingBuffer<MIMUFull, RINGBUFFER_CAPACITY> *>(g_ring)->write(sample);
  } else {
    MIMURaw sample = {
      d.data.ax,
      d.data.ay,
      d.data.az,
      d.data.gx,
      d.data.gy,
      d.data.gz
    };
    reinterpret_cast<InterCoreRingBuffer<MIMURaw, RINGBUFFER_CAPACITY> *>(g_ring)->write(sample);
  }
}
