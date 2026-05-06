/*
 *  Bmi160Core.ino - BMI160 sensor core for ManySensor.
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
#include <BMI160Gen.h>
#include <InterCoreRingBuffer.h>
#include <MultiCoreSpinLock.h>
#include "ManySensorShared.h"

static constexpr float BMI_ACC_G_TO_MPS2 = 9.80665f;
static constexpr float BMI_GYRO_DPS_TO_RADPS = 0.01745329252f;

static void *g_ring = NULL;
static MultiCoreSpinLock *g_i2c_lock = NULL;
static SensorState g_state = SENSOR_STATE_READY;
static uint32_t g_mode = BMI160_MODE_ACC;

static void convertAccelToMps2(float &ax, float &ay, float &az)
{
  ax *= BMI_ACC_G_TO_MPS2;
  ay *= BMI_ACC_G_TO_MPS2;
  az *= BMI_ACC_G_TO_MPS2;
}

static void convertGyroToRadps(float &gx, float &gy, float &gz)
{
  gx *= BMI_GYRO_DPS_TO_RADPS;
  gy *= BMI_GYRO_DPS_TO_RADPS;
  gz *= BMI_GYRO_DPS_TO_RADPS;
}

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
  BMI160.begin();
  BMI160.setAccelerometerRange(2);
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
    case MSG_SET_SHARED_BMI:
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
      haltOnError(SENSOR_ERR_BMI_UNEXPECTED_MSG);
    }
  }

  if (g_state != SENSOR_STATE_RUN || !g_ring || !g_i2c_lock) {
    return;
  }

  if (!MultiCoreSpin::acquire(g_i2c_lock, 3000)) {
    haltOnError(SENSOR_ERR_BMI_LOCK_TIMEOUT);
  }

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  up_enable_irq(CXD56_IRQ_SCU_I2C0);
  if (g_mode == BMI160_MODE_ACC) {
    BMI160.readAccelerometerScaled(ax, ay, az);
    convertAccelToMps2(ax, ay, az);
  } else if (g_mode == BMI160_MODE_GYRO) {
    BMI160.readGyroScaled(gx, gy, gz);
    convertGyroToRadps(gx, gy, gz);
  } else {
    BMI160.readAccelerometerScaled(ax, ay, az);
    BMI160.readGyroScaled(gx, gy, gz);
    convertAccelToMps2(ax, ay, az);
    convertGyroToRadps(gx, gy, gz);
  }
  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  if (g_mode == BMI160_MODE_ACC) {
    BMI160Acc sample = { ax, ay, az };
    reinterpret_cast<InterCoreRingBuffer<BMI160Acc, RINGBUFFER_CAPACITY> *>(g_ring)->write(sample);
  } else if (g_mode == BMI160_MODE_GYRO) {
    BMI160Gyro sample = { gx, gy, gz };
    reinterpret_cast<InterCoreRingBuffer<BMI160Gyro, RINGBUFFER_CAPACITY> *>(g_ring)->write(sample);
  } else {
    BMI160Imu sample = { ax, ay, az, gx, gy, gz };
    reinterpret_cast<InterCoreRingBuffer<BMI160Imu, RINGBUFFER_CAPACITY> *>(g_ring)->write(sample);
  }

  MultiCoreSpin::release(g_i2c_lock);
}
