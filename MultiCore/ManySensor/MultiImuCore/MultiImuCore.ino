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
#include "SpresenseIMU.h"
#include "ManySensorShared.h"

#define SAMPLINGRATE      (1920)
#define ADRANGE           (4)
#define GDRANGE           (500)
#define FIFO_DEPTH        (1)

static float *g_data = NULL;  /* DATA: [ax, ay, az, gx, gy, gz], FULL: [ts_s, temp_c, ax, ay, az, gx, gy, gz] */
static uint32_t g_state = SENSOR_STATE_READY;
static uint32_t g_mode = MIMU_MODE_DATA;

static void haltOnError(int32_t errCode)
{
  (void)errCode;
  while (1) {
    delay(1000);
  }
}

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
  MP.RecvTimeout(3000);
}

void loop() {
  int8_t msgid;
  uint32_t phys_addr;

  /* Receive shared address on first call */
  if (!g_data) {
    int ret = MP.Recv(&msgid, &phys_addr, 0);
    if (ret < 0 || msgid != MSG_SET_SHARED_IMU) {
      delay(100);
      return;
    }
    g_data = reinterpret_cast<float *>(phys_addr);
  }

  /* Wait for measurement request */
  int ret = MP.Recv(&msgid, &phys_addr, 0);
  if (ret < 0) {
    haltOnError(SENSOR_ERR_MIMU_RECV_FAILED);
  }

  switch (msgid) {
  case MSG_SET_SHARED_IMU:
    g_data = reinterpret_cast<float *>(phys_addr);
    return;

  case MSG_SENSOR_STOP:
    g_state = SENSOR_STATE_READY;
    MP.Send(MSG_RET_STATE, SENSOR_ERROR_OK);
    return;

  case MSG_SENSOR_START:
    g_state = SENSOR_STATE_RUN;
    g_mode = (uint32_t)phys_addr;  /* phys_addr carries MIMU mode */
    MP.Send(MSG_RET_STATE, SENSOR_ERROR_OK);
    return;

  case MSG_REQ_MIMU:
    if (g_state != SENSOR_STATE_RUN) {
      /* Sensor not running: ignore this request */
      return;
    }
    break;

  default:
    haltOnError(SENSOR_ERR_MIMU_UNEXPECTED_MSG);
  }

  /* Read sensor and write to shared memory */
  pwbImuData d;

  if (!SpresenseIMU.get(d)) {
    /* Data not available, ignore this request */
    return;
  }

  if (g_mode == MIMU_MODE_FULL) {
    g_data[0] = d.data.timestamp / 19200000.0f;
    g_data[1] = d.data.temp;
    g_data[2] = d.data.ax;
    g_data[3] = d.data.ay;
    g_data[4] = d.data.az;
    g_data[5] = d.data.gx;
    g_data[6] = d.data.gy;
    g_data[7] = d.data.gz;
  } else {
    g_data[0] = d.data.ax;
    g_data[1] = d.data.ay;
    g_data[2] = d.data.az;
    g_data[3] = d.data.gx;
    g_data[4] = d.data.gy;
    g_data[5] = d.data.gz;
  }

  /* Send reply */
  if (MP.Send(MSG_RET_MIMU, SENSOR_ERROR_OK) < 0) {
    haltOnError(SENSOR_ERR_MIMU_REPLY_SEND_FAILED);
  }
}
