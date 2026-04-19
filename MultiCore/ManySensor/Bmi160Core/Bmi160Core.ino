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
#include "ManySensorShared.h"

static constexpr float BMI_ACC_G_TO_MPS2 = 9.80665f;
static constexpr float BMI_GYRO_DPS_TO_RADPS = 0.01745329252f;

static float *g_data = NULL;  /* [ax, ay, az] / [gx, gy, gz] / [ax, ay, az, gx, gy, gz] */
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
    delay(1000);
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
  MP.RecvTimeout(3000);
}

// ------------------------------------------------------------
// LOOP (wait for main → measure → return)
// ------------------------------------------------------------
void loop() {

  int8_t msgid;
  uint32_t phys_addr;

  /* Receive shared address on first call */
  if (!g_data) {
    int ret = MP.Recv(&msgid, &phys_addr, 0);
    if (ret < 0 || msgid != MSG_SET_SHARED_BMI) {
      delay(100);
      return;
    }
    g_data = reinterpret_cast<float *>(phys_addr);
  }

  /* Wait for measurement request */
  int ret = MP.Recv(&msgid, &phys_addr, 0);

  if (ret < 0) {
    haltOnError(SENSOR_ERR_BMI_RECV_FAILED);
  }

  switch (msgid) {
  case MSG_SET_SHARED_BMI:
    g_data = reinterpret_cast<float *>(phys_addr);
    return;

  case MSG_SENSOR_STOP:
    g_state = SENSOR_STATE_READY;
    MP.Send(MSG_RET_STATE, SENSOR_ERROR_OK);
    return;

  case MSG_SENSOR_START:
    g_state = SENSOR_STATE_RUN;
    g_mode = (uint32_t)phys_addr;  /* phys_addr carries BMI160 mode */
    MP.Send(MSG_RET_STATE, SENSOR_ERROR_OK);
    return;

  case MSG_REQ_BMI:
    if (g_state != SENSOR_STATE_RUN) {
      /* Sensor not running: ignore this request */
      return;
    }
    break;

  default:
    haltOnError(SENSOR_ERR_BMI_UNEXPECTED_MSG);
  }

  /* Read sensor and write to shared memory based on selected mode */
  float ax, ay, az;
  float gx, gy, gz;

  up_enable_irq(CXD56_IRQ_SCU_I2C0);
  if (g_mode == BMI160_MODE_ACC) {
    BMI160.readAccelerometerScaled(ax, ay, az);
    convertAccelToMps2(ax, ay, az);
    g_data[0] = ax;
    g_data[1] = ay;
    g_data[2] = az;
  } else if (g_mode == BMI160_MODE_GYRO) {
    BMI160.readGyroScaled(gx, gy, gz);
    convertGyroToRadps(gx, gy, gz);
    g_data[0] = gx;
    g_data[1] = gy;
    g_data[2] = gz;
  } else {
    BMI160.readAccelerometerScaled(ax, ay, az);
    BMI160.readGyroScaled(gx, gy, gz);
    convertAccelToMps2(ax, ay, az);
    convertGyroToRadps(gx, gy, gz);
    g_data[0] = ax;
    g_data[1] = ay;
    g_data[2] = az;
    g_data[3] = gx;
    g_data[4] = gy;
    g_data[5] = gz;
  }
  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  /* Send reply */
  if (MP.Send(MSG_RET_BMI, SENSOR_ERROR_OK) < 0) {
    haltOnError(SENSOR_ERR_BMI_REPLY_SEND_FAILED);
  }
}
