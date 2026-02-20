/*
 *  evaluation.ino - SoftwareSerial library evaluation code.
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

#include <SoftwareSerial.h>

// ===== Settings =====
static const uint32_t BAUD = 115200;
static const uint32_t SEND_INTERVAL_MS = 1000;
static const int TX_PER_FRAME = 10;

// SoftwareSerial pins (main board only)
static const uint8_t SW_RX_PIN = 21;
static const uint8_t SW_TX_PIN = 20;

SoftwareSerial sw(SW_RX_PIN, SW_TX_PIN);

struct FrameConfig {
  uint16_t cfg;
  const char* name;
};

static const FrameConfig FRAMES[] = {
  { SERIAL_8N1, "8N1" },
  { SERIAL_8N2, "8N2" },
  { SERIAL_8E1, "8E1" },
  { SERIAL_8E2, "8E2" },
  { SERIAL_8O1, "8O1" },
  { SERIAL_8O2, "8O2" },
};

static const size_t FRAME_COUNT = sizeof(FRAMES) / sizeof(FRAMES[0]);

uint32_t lastSendMs = 0;
size_t frameIndex = 0;
int txInCurrentFrame = 0;
bool sendFromHw = true;
char hwToken = 'A';
char swToken = 'a';
int warmupTxRemaining = 0;

static char hwRxLine[128];
static int hwRxPos = 0;
static char swRxLine[128];
static int swRxPos = 0;

static char nextUpper(char c) {
  return (c == 'Z') ? 'A' : (char)(c + 1);
}

static char nextLower(char c) {
  return (c == 'z') ? 'a' : (char)(c + 1);
}

static void applyFrame(size_t index) {
  Serial2.end();
  sw.end();

  delay(10);

  Serial2.begin(BAUD, FRAMES[index].cfg);
  sw.begin(BAUD, FRAMES[index].cfg);

  delay(10);

  while (Serial2.available() > 0) {
    (void)Serial2.read();
  }
  while (sw.available() > 0) {
    (void)sw.read();
  }

  Serial.print("Active frame: ");
  Serial.println(FRAMES[index].name);

  warmupTxRemaining = 2;
}

static void printLine(const char* src, const char* line) {
  Serial.print(src);
  Serial.print(" ");
  Serial.println(line);
}

static void processRxByte(char c, char* buf, int* pos, const char* srcTag) {
  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    buf[*pos] = '\0';
    if (*pos > 0) {
      printLine(srcTag, buf);
    }
    *pos = 0;
    return;
  }

  if (*pos < 127) {
    buf[*pos] = c;
    (*pos)++;
  } else {
    buf[*pos] = '\0';
    printLine(srcTag, buf);
    *pos = 0;
  }
}

static void drainRx() {
  while (Serial2.available() > 0) {
    int v = Serial2.read();
    if (v >= 0) {
      processRxByte((char)v, hwRxLine, &hwRxPos, "[from HW]");
    }
  }

  while (sw.available() > 0) {
    int v = sw.read();
    if (v >= 0) {
      processRxByte((char)v, swRxLine, &swRxPos, "[from SW]");
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println("\n=== Spresense HW <-> SW Serial Check ===");
  Serial.println("MODE: LINE_LOG_V2");
  Serial.println("Wiring:");
  Serial.println("  1) Serial2 TX(D1) -> SW RX(pin 21)");
  Serial.println("  2) SW TX(pin 20) -> Serial2 RX(D0)");
  Serial.println("  3) Common GND (same board usually already shared)");
  Serial.println("----------------------------------------");

  Serial.print("Started. BAUD=");
  Serial.print(BAUD);
  Serial.print(", rotate each ");
  Serial.print(TX_PER_FRAME);
  Serial.println(" tx");

  applyFrame(frameIndex);
}

void loop() {
  // Keep reading both directions and print full lines
  drainRx();

  // Send one direction at a time
  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();

    if (warmupTxRemaining > 0) {
      if (sendFromHw) {
        Serial2.println("WARMUP");
        Serial.println("TX warmup: Serial2 -> SW");
      } else {
        sw.println("WARMUP");
        Serial.println("TX warmup: SW -> Serial2");
      }
      warmupTxRemaining--;
    } else {
      if (sendFromHw) {
        Serial2.print("HW from ");
        Serial2.println(hwToken);
        hwToken = nextUpper(hwToken);
        Serial.println("TX: Serial2 -> SW");
      } else {
        sw.print("SW from ");
        sw.println(swToken);
        swToken = nextLower(swToken);
        Serial.println("TX: SW -> Serial2");
      }

      txInCurrentFrame++;
    }

    sendFromHw = !sendFromHw;

    if (txInCurrentFrame >= TX_PER_FRAME) {
      Serial.println();
      txInCurrentFrame = 0;
      frameIndex = (frameIndex + 1) % FRAME_COUNT;
      applyFrame(frameIndex);
    }
  }
}
