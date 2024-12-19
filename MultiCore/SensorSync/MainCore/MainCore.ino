/*
 *  MainCore.ino - MultiCore Example to data syncronize
 *  Copyright 2024 Spresense Users
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

#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <RTC.h>

int analog_core = 1;
int i2c_core = 2;

void setup()
{
  int ret = 0;

  Serial.begin(115200);
  while (!Serial);

  // Initialize RTC at first
  RTC.begin();

  // Set the temporary RTC time
  RtcTime compiledDateTime(__DATE__, __TIME__);
  RTC.setTime(compiledDateTime);
  
  /* Launch SubCore1 */
  ret = MP.begin(analog_core);
  if (ret < 0) {
    printf("MP.begin%d error = %d\n", analog_core, ret);
  }

  /* Launch SubCore2 */
  ret = MP.begin(i2c_core);
  if (ret < 0) {
    printf("MP.begin%d error = %d\n", i2c_core, ret);
  }

  MP.RecvTimeout(MP_RECV_POLLING);

}

struct ResultAna {
  uint8_t   min;
  uint8_t   sec;
  uint16_t  msec;
  uint32_t  data;
};

struct ResultI2c {
  uint8_t   min;
  uint8_t   sec;
  uint16_t  msec;
  float     data;
};

void loop()
{
  int        ret;
  ResultAna* result_a;
  ResultI2c* result_i;
  int8_t     rcvid;

  ret = MP.Recv(&rcvid, &result_a, analog_core);
  if (ret > 0) {
    printf("analog data = %d (%d.%d.%d)\n", result_a->data, result_a->min, result_a->sec, result_a->msec);
  }

  ret = MP.Recv(&rcvid, &result_i, i2c_core);
  if (ret > 0) {
    printf("analog data = %f (%d.%d.%d)\n", result_i->data, result_i->min, result_i->sec, result_i->msec);
  }

}

