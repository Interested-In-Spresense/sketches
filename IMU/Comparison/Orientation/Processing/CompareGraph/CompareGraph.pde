import processing.serial.*;

final int BAUDRATE = 115200;
final String DEFAULT_PORT = "COM30";
final int HISTORY = 1400;

Serial serialPort;
String selectedPort = DEFAULT_PORT;
String statusText = "Press C to reconnect";

float[] bmiRoll = new float[HISTORY];
float[] bmiPitch = new float[HISTORY];
float[] bmiYaw = new float[HISTORY];
float[] mimuRoll = new float[HISTORY];
float[] mimuPitch = new float[HISTORY];
float[] mimuYaw = new float[HISTORY];

int head = 0;
boolean filled = false;
int bmiCount = 0;
int mimuCount = 0;
int elapsedBaseMillis = 0;
float displayRangeDeg = 180.0f;

// Logging
ArrayList<String> logBuffer = new ArrayList<String>();
final int LOG_BUFFER_SIZE = 25;
boolean verboseLogging = false;
float[] lastBmiAngles = {0, 0, 0};
float[] lastMimuAngles = {0, 0, 0};
final float DISCONTINUITY_THRESHOLD = 30.0f; // degrees

void setup() {
  size(1500, 900);
  frameRate(60);
  textFont(createFont("Consolas", 14));
  elapsedBaseMillis = millis();
  resetSeries();
  connectSerial();
}

void draw() {
  background(16);
  drawHeader();

  int margin = 22;
  int topY = 118;
  int panelW = width - margin * 2 - 320;
  int panelH = (height - topY - margin * 2) / 2;

  drawEulerPanel("BMI Euler (roll, pitch, yaw)", bmiRoll, bmiPitch, bmiYaw, margin, topY, panelW, panelH);
  drawEulerPanel("MIMU Euler (roll, pitch, yaw)", mimuRoll, mimuPitch, mimuYaw, margin, topY + margin + panelH, panelW, panelH);

  // Draw elapsed time and log panel on the right side
  int logX = panelW + margin + margin;
  int elapsedAreaH = 90;
  drawElapsedTime(logX, topY, 300, elapsedAreaH);
  drawLogPanel(logX, topY + elapsedAreaH, 300, height - topY - margin - elapsedAreaH);
}

void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');
  if (line == null) return;
  line = trim(line);
  if (line.length() == 0) return;

  if (line.startsWith("[BMI ]")) {
    float[] q = parseQuaternionLine(line);
    if (q != null) {
      if (verboseLogging) {
        addLog(String.format("BMI RX: Q0=%.4f Q1=%.4f Q2=%.4f Q3=%.4f", q[0], q[1], q[2], q[3]));
      }
      appendSample(true, q[0], q[1], q[2], q[3]);
      bmiCount++;
    }
    return;
  }

  if (line.startsWith("[MIMU]")) {
    float[] q = parseQuaternionLine(line);
    if (q != null) {
      if (verboseLogging) {
        addLog(String.format("MIMU RX: Q0=%.4f Q1=%.4f Q2=%.4f Q3=%.4f", q[0], q[1], q[2], q[3]));
      }
      appendSample(false, q[0], q[1], q[2], q[3]);
      mimuCount++;
    }
    return;
  }
}

void keyPressed() {
  if (key == 'c' || key == 'C') {
    elapsedBaseMillis = millis();
    connectSerial();
  } else if (key == 'r' || key == 'R') {
    resetSeries();
    statusText = "Series reset";
  } else if (key == 'l' || key == 'L') {
    verboseLogging = !verboseLogging;
    addLog((verboseLogging ? "[VERBOSE ON]" : "[VERBOSE OFF]"));
    println("Verbose logging: " + (verboseLogging ? "ON" : "OFF"));
  } else if (key == '0') {
    setDisplayRange(180.0f);
  } else if (key == '1') {
    setDisplayRange(90.0f);
  } else if (key == '2') {
    setDisplayRange(45.0f);
  } else if (key == '3') {
    setDisplayRange(10.0f);
  }
}

void setDisplayRange(float r) {
  displayRangeDeg = r;
  statusText = "Display range set to [" + nf(-displayRangeDeg, 1, 0) + ", " + nf(displayRangeDeg, 1, 0) + "]";
}

void appendSample(boolean isBmi, float q0, float q1, float q2, float q3) {
  PVector e = quatToEuler(q0, q1, q2, q3);

  if (isBmi) {
    bmiRoll[head] = e.x;
    bmiPitch[head] = e.y;
    bmiYaw[head] = e.z;

    // Check for discontinuity
    float rollDiff = angleDiff(lastBmiAngles[0], e.x);
    float pitchDiff = angleDiff(lastBmiAngles[1], e.y);
    float yawDiff = angleDiff(lastBmiAngles[2], e.z);
    float maxDiff = max(rollDiff, pitchDiff, yawDiff);

    if (maxDiff > DISCONTINUITY_THRESHOLD) {
      addLog(String.format("[BMI JUMP] R:%.1f→%.1f Δ=%.1f P:%.1f→%.1f Δ=%.1f Y:%.1f→%.1f Δ=%.1f",
        lastBmiAngles[0], e.x, rollDiff, lastBmiAngles[1], e.y, pitchDiff, lastBmiAngles[2], e.z, yawDiff));
      println("BMI DISCONTINUITY: Roll=" + rollDiff + "° Pitch=" + pitchDiff + "° Yaw=" + yawDiff + "°");
    }

    lastBmiAngles[0] = e.x;
    lastBmiAngles[1] = e.y;
    lastBmiAngles[2] = e.z;

    head++;
    if (head >= HISTORY) {
      head = 0;
      filled = true;
    }
  } else {
    int idx = head - 1;
    if (idx < 0) idx = HISTORY - 1;
    mimuRoll[idx] = e.x;
    mimuPitch[idx] = e.y;
    mimuYaw[idx] = e.z;

    // Check for discontinuity
    float rollDiff = angleDiff(lastMimuAngles[0], e.x);
    float pitchDiff = angleDiff(lastMimuAngles[1], e.y);
    float yawDiff = angleDiff(lastMimuAngles[2], e.z);
    float maxDiff = max(rollDiff, pitchDiff, yawDiff);

    if (maxDiff > DISCONTINUITY_THRESHOLD) {
      addLog(String.format("[MIMU JUMP] R:%.1f→%.1f Δ=%.1f P:%.1f→%.1f Δ=%.1f Y:%.1f→%.1f Δ=%.1f",
        lastMimuAngles[0], e.x, rollDiff, lastMimuAngles[1], e.y, pitchDiff, lastMimuAngles[2], e.z, yawDiff));
      println("MIMU DISCONTINUITY: Roll=" + rollDiff + "° Pitch=" + pitchDiff + "° Yaw=" + yawDiff + "°");
    }

    lastMimuAngles[0] = e.x;
    lastMimuAngles[1] = e.y;
    lastMimuAngles[2] = e.z;
  }
}

PVector quatToEuler(float q0, float q1, float q2, float q3) {
  float sinr = 2.0f * (q0 * q1 + q2 * q3);
  float cosr = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  float roll = atan2(sinr, cosr);

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  float pitch;
  if (abs(sinp) >= 1.0f) {
    pitch = copySign(HALF_PI, sinp);
  } else {
    pitch = asin(sinp);
  }

  float siny = 2.0f * (q0 * q3 + q1 * q2);
  float cosy = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  float yaw = atan2(siny, cosy);

  return new PVector(degrees(roll), degrees(pitch), degrees(yaw));
}

float copySign(float a, float b) {
  return (b >= 0.0f) ? abs(a) : -abs(a);
}

float[] parseQuaternionLine(String line) {
  float q0 = extractValue(line, "Q0=");
  float q1 = extractValue(line, "Q1=");
  float q2 = extractValue(line, "Q2=");
  float q3 = extractValue(line, "Q3=");
  if (Float.isNaN(q0) || Float.isNaN(q1) || Float.isNaN(q2) || Float.isNaN(q3)) {
    return null;
  }
  return new float[]{q0, q1, q2, q3};
}

float extractValue(String line, String key) {
  int keyPos = line.indexOf(key);
  if (keyPos < 0) return Float.NaN;

  int start = keyPos + key.length();
  int end = start;
  while (end < line.length()) {
    char ch = line.charAt(end);
    if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E') {
      end++;
    } else {
      break;
    }
  }

  if (end <= start) return Float.NaN;
  try {
    return Float.parseFloat(line.substring(start, end));
  } catch (Exception e) {
    return Float.NaN;
  }
}

void drawHeader() {
  fill(245);
  textSize(22);
  text("Euler Graph Compare", 22, 34);

  fill(180);
  textSize(14);
  text("Port: " + selectedPort + "   Baud: " + BAUDRATE + "   [C] reconnect  [R] reset  [L] log  [0/1/2/3] range", 22, 62);
  text("BMI lines: " + bmiCount + "   MIMU lines: " + mimuCount + "   Verbose: " + (verboseLogging ? "ON" : "OFF"), 22, 84);
  text(statusText, 22, 104);

  fill(230, 70, 70);
  rect(width - 320, 35, 16, 16);
  fill(230);
  text("Roll", width - 298, 48);

  fill(80, 210, 90);
  rect(width - 240, 35, 16, 16);
  fill(230);
  text("Pitch", width - 218, 48);

  fill(80, 120, 255);
  rect(width - 150, 35, 16, 16);
  fill(230);
  text("Yaw", width - 128, 48);
}

void drawEulerPanel(String label, float[] roll, float[] pitch, float[] yaw, int x, int y, int w, int h) {
  noFill();
  stroke(78);
  rect(x, y, w, h);

  stroke(40);
  for (int i = 1; i < 6; i++) {
    float gy = map(i, 0, 6, y, y + h);
    line(x, gy, x + w, gy);
  }

  float range = displayRangeDeg;
  drawSeries(roll, x, y, w, h, color(230, 70, 70), range);
  drawSeries(pitch, x, y, w, h, color(80, 210, 90), range);
  drawSeries(yaw, x, y, w, h, color(80, 120, 255), range);

  fill(240);
  textSize(16);
  text(label + " [deg] range[" + nf(-range, 1, 0) + "," + nf(range, 1, 0) + "]", x + 10, y + 24);

  float r = latest(roll);
  float p = latest(pitch);
  float yw = latest(yaw);
  fill(180);
  textSize(13);
  text("Roll=" + nf(r, 1, 2) + "  Pitch=" + nf(p, 1, 2) + "  Yaw=" + nf(yw, 1, 2), x + 10, y + h - 12);
}

void drawSeries(float[] series, int x, int y, int w, int h, int col, float range) {
  int n = filled ? HISTORY : head;
  if (n < 2) return;

  noFill();
  stroke(col);
  strokeWeight(1.3);
  beginShape();
  for (int i = 0; i < n; i++) {
    int idx = filled ? ((head + i) % HISTORY) : i;
    float v = constrain(series[idx], -range, range);
    float px = map(i, 0, max(1, n - 1), x + 1, x + w - 1);
    float py = map(v, -range, range, y + h - 1, y + 1);
    vertex(px, py);
  }
  endShape();
}

float latest(float[] series) {
  if (!filled && head == 0) return 0.0f;
  int idx = head - 1;
  if (idx < 0) idx = HISTORY - 1;
  return series[idx];
}

void resetSeries() {
  for (int i = 0; i < HISTORY; i++) {
    bmiRoll[i] = 0.0f;
    bmiPitch[i] = 0.0f;
    bmiYaw[i] = 0.0f;
    mimuRoll[i] = 0.0f;
    mimuPitch[i] = 0.0f;
    mimuYaw[i] = 0.0f;
  }

  head = 0;
  filled = false;
  bmiCount = 0;
  mimuCount = 0;
}

void connectSerial() {
  try {
    if (serialPort != null) {
      serialPort.stop();
      serialPort = null;
    }

    String[] ports = Serial.list();
    if (ports == null || ports.length == 0) {
      statusText = "No serial ports found";
      return;
    }

    int idx = findPortIndex(ports, selectedPort);
    if (idx < 0) {
      idx = 0;
      selectedPort = ports[idx];
    }

    serialPort = new Serial(this, ports[idx], BAUDRATE);
    serialPort.clear();
    serialPort.bufferUntil('\n');
    statusText = "Connected to " + ports[idx];
  } catch (Exception e) {
    statusText = "Connect failed: " + e.getMessage();
  }
}

int findPortIndex(String[] ports, String preferred) {
  for (int i = 0; i < ports.length; i++) {
    if (ports[i].equalsIgnoreCase(preferred)) return i;
  }
  return -1;
}

// ===== Logging Functions =====

void addLog(String message) {
  String timestamp = String.format("[%03d] ", millis() % 1000);
  logBuffer.add(timestamp + message);
  if (logBuffer.size() > LOG_BUFFER_SIZE) {
    logBuffer.remove(0);
  }
  println(message);
}

void drawLogPanel(int x, int y, int w, int h) {
  noFill();
  stroke(78);
  rect(x, y, w, h);

  fill(240);
  textSize(14);
  text("Log (Press L)", x + 8, y + 22);

  fill(180);
  textSize(11);
  int logY = y + 40;
  int lineHeight = 16;

  for (int i = 0; i < logBuffer.size(); i++) {
    if (logY + lineHeight > y + h - 8) break;
    String msg = (String)logBuffer.get(i);
    text(msg, x + 6, logY);
    logY += lineHeight;
  }
}

String formatElapsedHms(int elapsedMs) {
  int totalSec = max(0, elapsedMs / 1000);
  int hours = totalSec / 3600;
  int minutes = (totalSec % 3600) / 60;
  int seconds = totalSec % 60;
  return nf(hours, 2) + ":" + nf(minutes, 2) + ":" + nf(seconds, 2);
}

void drawElapsedTime(int x, int y, int w, int h) {
  int elapsedMs = millis() - elapsedBaseMillis;
  fill(180);
  textSize(64);
  text(formatElapsedHms(elapsedMs), x + 8, y + h - 24);
}

float angleDiff(float a1, float a2) {
  float diff = abs(a2 - a1);
  // Handle 180-degree wrapping
  if (diff > 180.0f) {
    diff = 360.0f - diff;
  }
  return diff;
}
