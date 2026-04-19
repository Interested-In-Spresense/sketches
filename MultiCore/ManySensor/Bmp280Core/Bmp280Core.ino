/*
 *  Bmp280Core.ino - BMP280 sensor core for ManySensor.
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
#include <Adafruit_BMP280.h>
#include "ManySensorShared.h"

Adafruit_BMP280 bmp; // use I2C interface

static float *g_data = NULL;  /* [temp_c, pressure_pa, altitude_m] or [pressure_pa] */
static uint32_t g_state = SENSOR_STATE_READY;
static uint32_t g_mode = BMP280_MODE_FULL;

static void haltOnError(int32_t errCode)
{
  (void)errCode;
  while (1) {
    delay(1000);
  }
}

// ------------------------------------------------------
// Setup
// ------------------------------------------------------
void setup()
{
  unsigned status = bmp.begin(0x76);
  if (!status) {
    haltOnError(SENSOR_ERR_BMP_BEGIN_FAILED);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  MP.begin();
  MP.RecvTimeout(3000);
}

// ------------------------------------------------------
// Loop (wait for main → measure → return)
// ------------------------------------------------------
void loop()
{
  int8_t msgid;
  uint32_t phys_addr;

  /* Receive shared address on first call */
  if (!g_data) {
    int ret = MP.Recv(&msgid, &phys_addr, 0);
    if (ret < 0 || msgid != MSG_SET_SHARED_BMP) {
      delay(100);
      return;
    }
    g_data = reinterpret_cast<float *>(phys_addr);
  }

  /* Wait for measurement request */
  int ret = MP.Recv(&msgid, &phys_addr, 0);
  if (ret < 0) {
    haltOnError(SENSOR_ERR_BMP_RECV_FAILED);
  }

  switch (msgid) {
  case MSG_SET_SHARED_BMP:
    g_data = reinterpret_cast<float *>(phys_addr);
    return;

  case MSG_SENSOR_STOP:
    g_state = SENSOR_STATE_READY;
    if (MP.Send(MSG_RET_STATE, SENSOR_ERROR_OK) < 0) {
      haltOnError(SENSOR_ERR_BMP_REPLY_SEND_FAILED);
    }
    return;

  case MSG_SENSOR_START:
    g_state = SENSOR_STATE_RUN;
    g_mode = (uint32_t)phys_addr;  /* phys_addr carries BMP280 mode */
    if (MP.Send(MSG_RET_STATE, SENSOR_ERROR_OK) < 0) {
      haltOnError(SENSOR_ERR_BMP_REPLY_SEND_FAILED);
    }
    return;

  case MSG_REQ_BMP:
    if (g_state != SENSOR_STATE_RUN) {
      /* Sensor not running: ignore this request */
      return;
    }
    break;

  default:
    haltOnError(SENSOR_ERR_BMP_UNEXPECTED_MSG);
  }

  /* Read sensor and write to shared memory */
  up_enable_irq(CXD56_IRQ_SCU_I2C0);

  if (g_mode == BMP280_MODE_FULL) {
    /* Write: temperature, pressure, altitude */
    g_data[0] = bmp.readTemperature();
    g_data[1] = bmp.readPressure();
    g_data[2] = bmp.readAltitude(1013.25);
  } else {
    /* Write: pressure only */
    g_data[0] = bmp.readPressure();
  }

  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  /* Send reply */
  if (MP.Send(MSG_RET_BMP, SENSOR_ERROR_OK) < 0) {
    haltOnError(SENSOR_ERR_BMP_REPLY_SEND_FAILED);
  }
}
