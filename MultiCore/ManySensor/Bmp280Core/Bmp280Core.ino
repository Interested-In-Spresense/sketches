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
#include <MultiCoreSpinLock.h>

Adafruit_BMP280 bmp; // use I2C interface

static float *g_data = NULL;  /* [temp_c, pressure_pa, altitude_m] or [pressure_pa] */
static MultiCoreSpinLock *g_i2c_lock = NULL;
static uint32_t g_state = SENSOR_STATE_READY;
static uint32_t g_mode = BMP280_MODE_FULL;

static void haltOnError(int32_t errCode)
{
  (void)errCode;
  while (1) {
    usleep(1000 * 1000);
  }
}

// ------------------------------------------------------
// Setup
// ------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.println("[BMP280] setup starting...");
  unsigned status = bmp.begin(0x76);
  Serial.print("[BMP280] begin status: ");
  Serial.println(status);
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
  MP.RecvTimeout(10);
  MP.Send(MSG_SUBCORE_READY, (uint32_t)BMP280CORE_ID);
}

// ------------------------------------------------------
// Loop (wait for main → measure → return)
// ------------------------------------------------------
void loop()
{
  int8_t msgid;
  uint32_t phys_addr;

  int ret = MP.Recv(&msgid, &phys_addr, 0);
  if (ret >= 0) {
    switch (msgid) {
    case MSG_SET_SHARED_BMP:
      g_data = reinterpret_cast<float *>(phys_addr);
      return;

    case MSG_SET_LOCK:
      g_i2c_lock = reinterpret_cast<MultiCoreSpinLock *>(phys_addr);
      return;

    case MSG_SENSOR_STOP:
      g_state = SENSOR_STATE_READY;
      return;

    case MSG_SENSOR_START:
      g_state = SENSOR_STATE_RUN;
      g_mode = (uint32_t)phys_addr;
      return;

    default:
      haltOnError(SENSOR_ERR_BMP_UNEXPECTED_MSG);
    }
  }

  if (g_state != SENSOR_STATE_RUN) {
    return;
  }

  if (!g_data) {
    return;
  }

  /* Read sensor and write to shared memory */
  if (g_i2c_lock) {
    if (!MultiCoreSpin::acquire(g_i2c_lock, 3000)) {
      haltOnError(SENSOR_ERR_BMP_LOCK_TIMEOUT);
    }
  }

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

  if (g_i2c_lock) {
    MultiCoreSpin::release(g_i2c_lock);
  }
}
