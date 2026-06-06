/*
 *  Bmi270Core.ino - BMI270 sensor core for Comparison.
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

#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <InterCoreRingBuffer.h>
#include <MultiCoreSpinLock.h>
#include <BMI270_Arduino.h>
#include "ComparisonShared.h"

BMI270Class BMI270;

static constexpr float BMI_GYRO_DPS_TO_RADPS = 0.01745329252f;
static constexpr uint32_t BMI_SAMPLE_RATE_HZ = 100U;
static constexpr uint32_t BMI_RING_RATE_HZ = 10U;
static constexpr uint32_t BMI_DECIMATION_FACTOR = (BMI_SAMPLE_RATE_HZ / BMI_RING_RATE_HZ);

static void *g_ring = NULL;
static MultiCoreSpinLock *g_i2c_lock = NULL;
static SensorState g_state = SENSOR_STATE_READY;
static uint32_t g_sample_count = 0;
static float g_gyro_bias_x = 0.0f;
static float g_gyro_bias_y = 0.0f;
static float g_gyro_bias_z = 0.0f;

struct QuaternionState {
  float q0;
  float q1;
  float q2;
  float q3;

  QuaternionState operator*(const QuaternionState &other) const
  {
    QuaternionState result;
    result.q0 = q0 * other.q0 - q1 * other.q1 - q2 * other.q2 - q3 * other.q3;
    result.q1 = q0 * other.q1 + q1 * other.q0 + q2 * other.q3 - q3 * other.q2;
    result.q2 = q0 * other.q2 - q1 * other.q3 + q2 * other.q0 + q3 * other.q1;
    result.q3 = q0 * other.q3 + q1 * other.q2 - q2 * other.q1 + q3 * other.q0;
    return result;
  }
};

static int8_t configureAccAndGyro()
{
  bmi2_sens_config sens_config[2];
  sens_config[0].type = BMI2_ACCEL;
  sens_config[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  sens_config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  sens_config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  sens_config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

  sens_config[1].type = BMI2_GYRO;
  sens_config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_config[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  sens_config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  sens_config[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
  sens_config[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  int8_t ret = BMI270.set_sensor_config(sens_config, 2);
  if (ret != 0) {
    return ret;
  }

  ret = BMI270.sensor_enable(sens_list, 2);
  return ret;
}

static bool initBmi270()
{
  int8_t ret = BMI270.begin(BMI270_I2C, BMI2_I2C_SEC_ADDR);
  if (ret != 0) {
    return false;
  }

  ret = configureAccAndGyro();
  if (ret != 0) {
    return false;
  }

  return true;
}

static bool readSensor(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
  bmi2_sens_float sensor_data;
  int8_t ret = BMI270.bmi2_get_sensor_float(&sensor_data);
  if (ret != 0) {
    return false;
  }

  ax = sensor_data.acc.x;
  ay = sensor_data.acc.y;
  az = sensor_data.acc.z;

  gx = sensor_data.gyr.x;
  gy = sensor_data.gyr.y;
  gz = sensor_data.gyr.z;

  return true;
}

static void convertGyroToRadps(float &gx, float &gy, float &gz)
{
  gx *= BMI_GYRO_DPS_TO_RADPS;
  gy *= BMI_GYRO_DPS_TO_RADPS;
  gz *= BMI_GYRO_DPS_TO_RADPS;
}

static void normalizeQuaternion(QuaternionState &quat)
{
  float norm = sqrt(quat.q0 * quat.q0 + quat.q1 * quat.q1 + quat.q2 * quat.q2 + quat.q3 * quat.q3);
  if (norm > 0.001f) {
    quat.q0 /= norm;
    quat.q1 /= norm;
    quat.q2 /= norm;
    quat.q3 /= norm;
  }
}

static QuaternionState makeDeltaQuaternion(float gx, float gy, float gz, float dt)
{
  QuaternionState result;
  float omega = sqrt(gx * gx + gy * gy + gz * gz);
  if (omega > 0.000001f) {
    float half_angle = 0.5f * omega * dt;
    float scale = sin(half_angle) / omega;
    result.q0 = cos(half_angle);
    result.q1 = gx * scale;
    result.q2 = gy * scale;
    result.q3 = gz * scale;
  } else {
    result.q0 = 1.0f;
    result.q1 = 0.5f * gx * dt;
    result.q2 = 0.5f * gy * dt;
    result.q3 = 0.5f * gz * dt;
  }
  normalizeQuaternion(result);
  return result;
}

static void haltOnError(int32_t errCode)
{
  (void)errCode;
  while (1) usleep(1000 * 1000);
}

static void calibrateGyroBias(uint32_t ms)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  uint32_t count = 0;
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < ms) {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    up_enable_irq(CXD56_IRQ_SCU_I2C0);
    if (!readSensor(ax, ay, az, gx, gy, gz)) {
      up_disable_irq(CXD56_IRQ_SCU_I2C0);
      continue;
    }
    up_disable_irq(CXD56_IRQ_SCU_I2C0);

    convertGyroToRadps(gx, gy, gz);
    sum_x += gx;
    sum_y += gy;
    sum_z += gz;
    count++;
  }

  if (count > 0) {
    g_gyro_bias_x = (float)(sum_x / count);
    g_gyro_bias_y = (float)(sum_y / count);
    g_gyro_bias_z = (float)(sum_z / count);
  }
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup()
{
  if (!initBmi270()) {
    haltOnError(-1201);
  }

  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING);

  delay(1000);
  calibrateGyroBias(2000);

  int sret = MP.Send(MSG_SENSOR_READY, (uint32_t)0, 0);
  if (sret < 0) {
    haltOnError(sret);
  }
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
  static uint32_t prev_timestamp_us = 0;
  uint32_t timestamp_us = micros();

  up_enable_irq(CXD56_IRQ_SCU_I2C0);
  if (!readSensor(ax, ay, az, gx, gy, gz)) {
    up_disable_irq(CXD56_IRQ_SCU_I2C0);
    MultiCoreSpin::release(g_i2c_lock);
    return;
  }

  convertGyroToRadps(gx, gy, gz);
  gx -= g_gyro_bias_x;
  gy -= g_gyro_bias_y;
  gz -= g_gyro_bias_z;
  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  MultiCoreSpin::release(g_i2c_lock);

  static QuaternionState data = {1.0f, 0.0f, 0.0f, 0.0f};
  static int decimate_count = 0;

  float timestamp_s = timestamp_us / 1000000.0f;
  float dt = (prev_timestamp_us == 0)
             ? (1.0f / BMI_SAMPLE_RATE_HZ)
             : ((uint32_t)(timestamp_us - prev_timestamp_us) / 1000000.0f);
  prev_timestamp_us = timestamp_us;
  QuaternionState result = makeDeltaQuaternion(gx, gy, gz, dt);
  data = data * result;
  normalizeQuaternion(data);

  decimate_count++;

  if (decimate_count >= (int)BMI_DECIMATION_FACTOR) {
    BMIQuaternion sample = { timestamp_s, data.q0, data.q1, data.q2, data.q3 };
    reinterpret_cast<InterCoreRingBuffer<BMIQuaternion, RINGBUFFER_CAPACITY> *>(g_ring)->write(sample);
    decimate_count = 0;
  }

  g_sample_count++;
}