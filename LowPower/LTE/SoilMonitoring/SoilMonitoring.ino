/*
 *  SoilMonitoring.ino - SoilMonitoring on the ambient via LTE for Low-Power Sensing
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

#include <SoftwareSerial.h>
#include "SLT5006.h"

// RX=29, TX=30 on Spresense; ENA=2.
static const uint8_t RX_PIN = 29;
static const uint8_t TX_PIN = 30;
static const uint8_t ENABLE_PIN = 2;

SoftwareSerial sw(RX_PIN, TX_PIN);
SLT5006 Sensor(sw, ENABLE_PIN);

/* Change the settings according to your sim card */
#define APN_NAME "meeq.io"
#define APN_USRNAME "meeq"
#define APN_PASSWD "meeq"

/* Change the settings accroding to your ambient channel */
#define AMBI_CHANNEL XXXXXXXXX
#define AMBI_WRITEKEY "XXXXXXXXXXXXXXX"

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

  Serial.println("SLT5006 sample_softserial start");

  Sensor.begin(ENABLE_PIN);
  Sensor.enable();

  if (!doAttach()) {
    Serial.println("LTE attach failed");
    theAmbient.end();
    lteAccess.shutdown();
    while(1);
  }

  uint8_t major, minor, rev;
  if (Sensor.version(major, minor, rev)) {
    Serial.print("FW version: ");
    Serial.print(major); Serial.print(".");
    Serial.print(minor); Serial.print(".");
    Serial.println(rev);
  } else {
    Serial.println("version get failed");
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

  if (!Sensor.send()) {
    Serial.println("send failed");
    delay(1000);
    return;
  }

  if (!Sensor.receive(5000)) {
    Serial.println("receive failed");
    delay(1000);
    return;
  }

  float t = Sensor.temp();
  float v = Sensor.vwc();
  float e = Sensor.ec_pore();

  theAmbient.set(1, t);
  theAmbient.set(2, v);
  theAmbient.set(3, e);
  bool ret = theAmbient.send();

  Serial.print("Temp[degC]=");
  Serial.print(t, 4);
  Serial.print(", VWC[%]=");
  Serial.print(v, 1);
  Serial.print(", EC_PORE[dS/m]=");
  Serial.println(e, 3);

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
