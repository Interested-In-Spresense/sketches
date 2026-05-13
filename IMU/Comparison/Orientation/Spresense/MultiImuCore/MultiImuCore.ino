/*
 *  MultiImuCore.ino - MultiIMU sensor core for Comparison.
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

#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <InterCoreRingBuffer.h>
#include <MultiCoreSpinLock.h>
#include "SpresenseIMU.h"
#include "ComparisonShared.h"

#define SAMPLINGRATE      (1920)
#define ADRANGE           (4)
#define GDRANGE           (500)
#define FIFO_DEPTH        (1)

static constexpr float TIMESTAMP_WRAP_SECONDS = 4294967296.0f / 19200000.0f;

static void *g_ring = NULL;
static MultiCoreSpinLock *g_i2c_lock = NULL;
static uint32_t g_state = SENSOR_STATE_READY;
static uint32_t g_sample_count = 0;
static float g_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

static void haltOnError(int32_t errCode)
{
  (void)errCode;
  while (1) usleep(1000 * 1000);
}

static void calibrateGyroBias(uint32_t ms)
{
  double sum[3] = {0.0, 0.0, 0.0};
  uint32_t count = 0;
  uint32_t start = millis();
  pwbImuData s;

  while ((uint32_t)(millis() - start) < ms) {
    if (SpresenseIMU.get(s)) {
      sum[0] += s.data.gx;
      sum[1] += s.data.gy;
      sum[2] += s.data.gz;
      count++;
    }
  }

  if (count > 0) {
    g_gyro_bias[0] = (float)(sum[0] / count);
    g_gyro_bias[1] = (float)(sum[1] / count);
    g_gyro_bias[2] = (float)(sum[2] / count);
  }
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING);

  int ret = SpresenseIMU.begin();
  if (ret < 0) { haltOnError(SENSOR_ERR_MIMU_BEGIN_FAILED); }

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret) { haltOnError(SENSOR_ERR_MIMU_INIT_FAILED); }

  ret = SpresenseIMU.start();
  if (!ret) { haltOnError(SENSOR_ERR_MIMU_START_FAILED); }

  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  delay(1000);
  calibrateGyroBias(2000);

  ret = MP.Send(MSG_SENSOR_READY, (uint32_t)0, 0);
  if (ret < 0) { haltOnError(ret); }
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
      return;
    default:
      haltOnError(-1);
    }
  }

  if (g_state != SENSOR_STATE_RUN || !g_ring) {
    return;
  }

  static int decimate_count = 0;
  static float last_ts = 0;
  static pwbQuaternionData data;
  pwbImuData d;

  if (!SpresenseIMU.get(d)) {
    return;
  }

  d.data.gx -= g_gyro_bias[0];
  d.data.gy -= g_gyro_bias[1];
  d.data.gz -= g_gyro_bias[2];

  pwbQuaternionData result;
  SpresenseIMU.convQuaternion(result, d.data, last_ts, false);
  data = data * result;

  float next_ts = d.data.timestamp / 19200000.0f;
  if (last_ts > 0.0f) {
    while (next_ts <= last_ts) {
      next_ts += TIMESTAMP_WRAP_SECONDS;
    }
  }
  last_ts = next_ts;

  decimate_count++;
  if (decimate_count >= 10) {
    MIMUQuaternion sample = {
      last_ts,
      d.data.temp,
      data.q0, data.q1, data.q2, data.q3
    };
    if (reinterpret_cast<InterCoreRingBuffer<MIMUQuaternion, RINGBUFFER_CAPACITY> *>(g_ring)->write(sample) == 0) {
      Serial.println("[MIMU-sub2] write failed (ring full)");
    }
    decimate_count = 0;

    g_sample_count++;
  }
}
