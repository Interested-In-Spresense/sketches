/*
 *  I2cCore.ino - MultiCore Example to data syncronize
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

#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <RTC.h>
#include <BMI160Gen.h>


void printClock(RtcTime &rtc)
{
  printf("%04d/%02d/%02d %02d:%02d:%02d:%08ld\n",
         rtc.year(), rtc.month(), rtc.day(),
         rtc.hour(), rtc.minute(), rtc.second(),rtc.nsec());
}

void setup()
{
  int ret = 0;

  ret = MP.begin();
  if (ret < 0) {
    errorLoop(2);
  }

  // Initialize RTC at first
  RTC.begin();

  //BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
//  BMI160.begin();

  // Set the accelerometer range to 2G
//  BMI160.setAccelerometerRange(2);

}

struct Result {
  uint8_t   min;
  uint8_t   sec;
  uint16_t  msec;
  float     data;
};

void loop()
{
  int      ret;
  int8_t   msgid = 10;
  Result   result;

  float ax, ay, az;   //scaled accelerometer values
  // read accelerometer measurements from device, scaled to the configured range
  BMI160.readAccelerometerScaled(ax, ay, az);
  result.data = ax;

  RtcTime now = RTC.getTime();
//  printClock(now);

  result.min  = now.minute();
  result.sec  = now.second();
  result.msec = now.nsec() / (1000*1000);

  ret = MP.Send(msgid, &result);
  if (ret < 0) {
    errorLoop(4);
  }

  usleep(500*1000);

}

void errorLoop(int num)
{
  int i;

  while (1) {
    for (i = 0; i < num; i++) {
      ledOn(LED0);
      delay(300);
      ledOff(LED0);
      delay(300);
    }
    delay(1000);
  }
}
