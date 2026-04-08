/*
 *  MultiImuCore.ino - MultiIMU subcore responder for ManySensor.
 *  MultiIMU reader (SpresenseIMU)
 */

#if (SUBCORE != 3)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include "SpresenseIMU.h"

#define SAMPLINGRATE      (1920)
#define ADRANGE           (4)
#define GDRANGE           (500)
#define FIFO_DEPTH        (1)

#define MSG_REQ_MIMU  5
#define MSG_RET_MIMU  6

void setup() {
  Serial.begin(115200);
  while (!Serial);

  int ret = SpresenseIMU.begin();
  if (ret < 0) {
    Serial.println("[Sub3] SpresenseIMU.begin failed");
  }

  ret = SpresenseIMU.initialize(SAMPLINGRATE, ADRANGE, GDRANGE, FIFO_DEPTH);
  if (!ret) {
    Serial.println("[Sub3] SpresenseIMU.initialize failed");
  }

  ret = SpresenseIMU.start();
  if (!ret) {
    Serial.println("[Sub3] SpresenseIMU.start failed");
  }

  up_disable_irq(CXD56_IRQ_SCU_I2C0);

  MP.begin();
  MP.RecvTimeout(3000);
}

void loop() {
  int8_t msgid;
  uint32_t dummy;

  if (MP.Recv(&msgid, &dummy) >= 0 && msgid == MSG_REQ_MIMU) {
    pwbImuData d;
    
    up_enable_irq(CXD56_IRQ_SCU_I2C0);
    
    if (SpresenseIMU.get(d)) {
      Serial.print("[Sub3] mimu acc:\t");
      Serial.print(d.data.ax);
      Serial.print("\t");
      Serial.print(d.data.ay);
      Serial.print("\t");
      Serial.print(d.data.az);
      Serial.print(" gyro:\t");
      Serial.print(d.data.gx);
      Serial.print("\t");
      Serial.print(d.data.gy);
      Serial.print("\t");
      Serial.println(d.data.gz);
      Serial.flush();
    } else {
      Serial.println("[Sub3] SpresenseIMU.get failed");
    }
    
    up_disable_irq(CXD56_IRQ_SCU_I2C0);
    MP.Send(MSG_RET_MIMU, (uint32_t)1);
    
  } else {
    Serial.println("[Sub3] recv timeout");
  }
}
