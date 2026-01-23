/*
 * ============================================================
 *  RumiCar example for Dual DC Motor Rover
 * ============================================================
 *
 *  Overview:
 *    This is a sample implementation of RumiCar using dual DC motors.
 *
 *    The rover continuously measures distances at the
 *    center, left, and right directions, and turns toward
 *    the side with more available space.
 *
 * ------------------------------------------------------------
 *  Hardware
 * ------------------------------------------------------------
 *    MCU        : Arduino-compatible board
 *    Sensors    : VL53L0X ~ 3 (I2C, single-shot ranging)
 *    Motors     : Dual DC Motor Rover (Rover.h)
 *
 *    XSHUT Pin Assignment:
 *      - Center sensor (sensor0) : GPIO 24
 *      - Left sensor   (sensor1) : GPIO 25
 *      - Right sensor  (sensor2) : GPIO 26
 *
 * ------------------------------------------------------------
 *  I2C Address Assignment
 * ------------------------------------------------------------
 *      sensor0 (center) : 0x14 (20)
 *      sensor1 (left)   : 0x15 (21)
 *      sensor2 (right)  : 0x16 (22)
 *
 *    XSHUT pins are used to enable sensors one by one
 *    so that each VL53L0X can be assigned a unique I2C address.
 *
 * ------------------------------------------------------------
 *  Control Logic
 * ------------------------------------------------------------
 *    - right > left  : rover turns LEFT
 *    - right < left  : rover turns RIGHT
 *
 *    Direction state is tracked using an enum to prevent
 *    redundant motor commands.
 *
 * ------------------------------------------------------------
 *  Build-Time Configuration Options
 * ------------------------------------------------------------
 *    LONG_RANGE
 *      - Extends sensing distance by increasing sensitivity.
 *      - More susceptible to reflections and noise.
 *
 *    HIGH_SPEED
 *      - Reduces measurement timing budget (20 ms).
 *
 *    HIGH_ACCURACY
 *      - Increases measurement timing budget (200 ms).
 *
 *    NOTE:
 *      Enable ONLY ONE of HIGH_SPEED or HIGH_ACCURACY.
 *
 * ------------------------------------------------------------
 *  Notes
 * ------------------------------------------------------------
 *    - Distance values are in millimeters (mm).
 *    - Sensor timeout is set to 500 ms.
 *    - Rover speed is set via rover.setSpeed().
 *
 * ============================================================
 */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor0;
VL53L0X sensor1;
VL53L0X sensor2;

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

#define HIGH_SPEED
//#define HIGH_ACCURACY

#include "Rover.h"

DualDCMotorRover rover;

#define XSHUT0 24
#define XSHUT1 25
#define XSHUT2 26

/* ============================================================ *
   Setup
 * ============================================================ */
void setup()
{
  rover.begin(true, 20, 21, 18, 19);

  pinMode(XSHUT0, OUTPUT);
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  digitalWrite(XSHUT0, LOW);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(150);

  Serial.begin(115200);
  Wire.begin();

  //sensor0
  digitalWrite(XSHUT0, HIGH);
  delay(150);
  sensor0.init(true);
  if(!sensor0.init()) { Serial.println("sensor1 init failed"); while(1){} }
  delay(100);
  sensor0.setAddress((uint8_t)20);
  sensor0.setTimeout(500);

  //sensor1
  digitalWrite(XSHUT1, HIGH);
  delay(150);
  sensor1.init(true);
  if(!sensor1.init()) { Serial.println("sensor1 init failed"); while(1){} }
  delay(100);
  sensor1.setAddress((uint8_t)21);
  sensor1.setTimeout(500);

  //sensor2
  digitalWrite(XSHUT2, HIGH);
  delay(150);
  sensor2.init(true);
  if(!sensor2.init()) { Serial.println("sensor2 init failed"); while(1){} }
  delay(100);
  sensor2.setAddress((uint8_t)22);
  sensor2.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
//  sensor0.setMeasurementTimingBudget(20000);
  sensor1.setMeasurementTimingBudget(20000);
  sensor2.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

  rover.setSpeed(100);
  rover.front();
  puts("front!");

}

/* ============================================================ *
   Loop
 * ============================================================ */

enum Direction {
  dNone = 0,
  dCenter,
  dLeft,
  dRight
};

void loop()
{

  static Direction state = dNone;

  int center = sensor0.readRangeSingleMillimeters();
  int left   = sensor1.readRangeSingleMillimeters();
  int right  = sensor2.readRangeSingleMillimeters();
  Serial.print("sensor0: ");
  Serial.println(center);
  Serial.print("sensor1: ");
  Serial.println(right);
  Serial.print("sensor2: ");
  Serial.println(left);
  if (sensor0.timeoutOccurred()) { Serial.print("sensor0 TIMEOUT"); }
  if (sensor1.timeoutOccurred()) { Serial.print("sensor1 TIMEOUT"); }
  if (sensor2.timeoutOccurred()) { Serial.print("sensor2 TIMEOUT"); }

  if(right>left) {
    ledOn(LED2);
    ledOff(LED0);
    if(state != dLeft){
      rover.left();
      puts("left!");
      state = dLeft;
    }
  }

  if(right<left) {
    ledOn(LED0);
    ledOff(LED2);
    if(state != dRight){
      rover.right();
      state = dRight;
      puts("right!");
    }
  }

  Serial.println();
}

