/*
 *  AmbientTemplate.ino - Ambient template via LTE for Low-Power Sensing
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

#include <LowPower.h>
#include <Arduino.h>
#include "Ambient_SpresenseLTEM.h"

/* Change the settings according to your sim card */
#define APN_NAME "meeq.io"
#define APN_USRNAME "meeq"
#define APN_PASSWD "meeq"

/* Change the settings accroding to your ambient channel */
#define AMBI_CHANNEL XXXXXXX
#define AMBI_WRITEKEY "XXXXXXXXXXXXXXXX"

// Deep sleep interval settings (adjust as needed)
#define LOW_POWER_SLEEP_SEC 3600
//#define LOW_POWER_SLEEP_SEC 3

LTE lteAccess;
LTEClient lteClient;
Ambient_SpresenseLTEM theAmbient(lteAccess, lteClient);

static bool doAttach()
{
  while (true) {
    if (lteAccess.begin() != LTE_SEARCHING) {
      Serial.println("Could not transition to LTE_SEARCHING.");
      return false;
    }

    if (lteAccess.attach(APN_NAME, APN_USRNAME, APN_PASSWD) == LTE_READY) {
      Serial.println("attach succeeded.");
      return true;
    }

    Serial.println("An error occurred, shutdown and try again.");
    lteAccess.shutdown();
    sleep(1);
  }
}

static void sleepForLowPower()
{
  Serial.print("Going to deep sleep for ");
  Serial.print(LOW_POWER_SLEEP_SEC);
  Serial.println(" sec.");
  theAmbient.end();
  lteAccess.shutdown();
  delay(100);
  LowPower.deepSleep(LOW_POWER_SLEEP_SEC);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Wake up!");

  // Initialize LowPower API before using deep sleep.
  LowPower.begin();
  Serial.println("lowpower begin!");

  if (!doAttach()) {
    Serial.println("LTE attach failed");
    theAmbient.end();
    lteAccess.shutdown();
    while(1);
  }

  if (!theAmbient.begin(AMBI_CHANNEL, AMBI_WRITEKEY)) {
    Serial.println("theAmbient begin failed");
    theAmbient.end();
    lteAccess.shutdown();
    while(1);
  }
  Serial.println("Ambient begin!");

}

void loop()
{
  unsigned long data = 0;
  while (data == 0) {
    data = lteAccess.getTime();
    if (data == 0) {
      sleep(1);
    }
  }

  theAmbient.set(1, data);
  bool ret = theAmbient.send();

  Serial.print("data=");
  Serial.println(data);

  if (ret == 0) {
      Serial.println("*** ERROR! LTE reboot! ***\n");
      theAmbient.end();
      lteAccess.shutdown();
      while(1) {
        Serial.println("Connecting Network...");
        if (doAttach()) {
          break;
        }
      }
  }

  sleep(3);

  sleepForLowPower();

}
