import processing.serial.*;

final int BAUDRATE = 115200;
final String DEFAULT_PORT = "COM30";

Serial serialPort;
String selectedPort = DEFAULT_PORT;
String statusText = "Press C to reconnect";

Quat bmiOrientation = new Quat(1, 0, 0, 0);
Quat mimuOrientation = new Quat(1, 0, 0, 0);

int bmiCount = 0;
int mimuCount = 0;
int elapsedBaseMillis = 0;

class Quat {
  float w;
  float x;
  float y;
  float z;

  Quat(float w_, float x_, float y_, float z_) {
    w = w_;
    x = x_;
    y = y_;
    z = z_;
  }

  void set(float w_, float x_, float y_, float z_) {
    w = w_;
    x = x_;
    y = y_;
    z = z_;
    normalize();
  }

  void normalize() {
    float n = sqrt(w * w + x * x + y * y + z * z);
    if (n <= 0.0f) {
      w = 1.0f;
      x = 0.0f;
      y = 0.0f;
      z = 0.0f;
      return;
    }
    w /= n;
    x /= n;
    y /= n;
    z /= n;
  }
}

void setup() {
  size(1420, 820, P3D);
  smooth(8);
  textFont(createFont("Consolas", 14));
  elapsedBaseMillis = millis();
  connectSerial();
}

void draw() {
  background(250);

  while (serialPort != null && serialPort.available() > 0) {
    String line = serialPort.readStringUntil('\n');
    if (line != null) {
      parseLine(trim(line));
    }
  }

  drawScene();
  drawHUD();
}

void keyPressed() {
  if (key == 'c' || key == 'C') {
    elapsedBaseMillis = millis();
    connectSerial();
  } else if (key == 'r' || key == 'R') {
    bmiOrientation.set(1, 0, 0, 0);
    mimuOrientation.set(1, 0, 0, 0);
    statusText = "Orientation reset";
  }
}

void parseLine(String line) {
  if (line == null || line.length() == 0) {
    return;
  }

  if (line.startsWith("[BMI ]")) {
    float q0 = extractValue(line, "Q0=");
    float q1 = extractValue(line, "Q1=");
    float q2 = extractValue(line, "Q2=");
    float q3 = extractValue(line, "Q3=");
    if (!Float.isNaN(q0) && !Float.isNaN(q1) && !Float.isNaN(q2) && !Float.isNaN(q3)) {
      bmiOrientation.set(q0, q1, q2, q3);
      bmiCount++;
    }
    return;
  }

  if (line.startsWith("[MIMU]")) {
    float q0 = extractValue(line, "Q0=");
    float q1 = extractValue(line, "Q1=");
    float q2 = extractValue(line, "Q2=");
    float q3 = extractValue(line, "Q3=");
    if (!Float.isNaN(q0) && !Float.isNaN(q1) && !Float.isNaN(q2) && !Float.isNaN(q3)) {
      mimuOrientation.set(q0, q1, q2, q3);
      mimuCount++;
    }
    return;
  }
}

float extractValue(String line, String key) {
  int keyPos = line.indexOf(key);
  if (keyPos < 0) {
    return Float.NaN;
  }

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

  if (end <= start) {
    return Float.NaN;
  }

  try {
    return Float.parseFloat(line.substring(start, end));
  } catch (Exception e) {
    return Float.NaN;
  }
}

void drawScene() {
  lights();

  drawModelPanel(width * 0.32f, height * 0.50f, 0, bmiOrientation, color(255, 145, 60), "BMI160", -18);
  drawModelPanel(width * 0.68f, height * 0.50f, 0, mimuOrientation, color(70, 170, 255), "Multi-IMU", 18);
}

void drawModelPanel(float cx, float cy, float cz, Quat q, int col, String label, float labelOffsetX) {
  pushMatrix();
  translate(cx, cy, cz);

  stroke(180);
  noFill();
  box(260, 260, 260);

  strokeWeight(2);
  drawWorldAxis(150);

  applyRotationMatrix(q);
  drawDeviceModel(col);

  popMatrix();

  fill(30);
  textSize(30);
  textAlign(CENTER, CENTER);
  text(label, cx + labelOffsetX, cy - 188);
}

void drawDeviceModel(int col) {
  noStroke();

  pushMatrix();
  fill(col);
  box(92, 24, 160);
  popMatrix();

  pushMatrix();
  translate(0, -10, 63);
  fill(45, 45, 45);
  box(26, 10, 26);
  popMatrix();

  pushMatrix();
  translate(0, -10, -63);
  fill(45, 45, 45);
  box(26, 10, 26);
  popMatrix();

  strokeWeight(4);
  stroke(220, 40, 40);
  line(0, 0, 0, 102, 0, 0);
  stroke(40, 180, 70);
  line(0, 0, 0, 0, 0, 102);
  stroke(40, 80, 220);
  line(0, 0, 0, 0, -102, 0);
}

void drawWorldAxis(float len) {
  stroke(230, 70, 70);
  line(0, 0, 0, len, 0, 0);
  stroke(70, 190, 90);
  line(0, 0, 0, 0, 0, len);
  stroke(70, 110, 230);
  line(0, 0, 0, 0, -len, 0);
}

void applyRotationMatrix(Quat q) {
  float[] m = remapMatrixSwapYZ(quatToMatrix3(q));
  applyMatrix(
    m[0], m[1], m[2], 0,
    m[3], m[4], m[5], 0,
    m[6], m[7], m[8], 0,
    0,    0,    0,    1
  );
}

float[] remapMatrixSwapYZ(float[] m) {
  return new float[] {
    m[0], m[2], m[1],
    m[6], m[8], m[7],
    m[3], m[5], m[4]
  };
}

float[] quatToMatrix3(Quat q) {
  float w = q.w;
  float x = q.x;
  float y = q.y;
  float z = q.z;

  float xx = x * x;
  float yy = y * y;
  float zz = z * z;
  float xy = x * y;
  float xz = x * z;
  float yz = y * z;
  float wx = w * x;
  float wy = w * y;
  float wz = w * z;

  return new float[] {
    1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),        2.0f * (xz + wy),
    2.0f * (xy + wz),        1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
    2.0f * (xz - wy),        2.0f * (yz + wx),        1.0f - 2.0f * (xx + yy)
  };
}

PVector quatToEulerDeg(Quat q) {
  float sinr = 2.0f * (q.w * q.x + q.y * q.z);
  float cosr = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  float roll = atan2(sinr, cosr);

  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  float pitch;
  if (abs(sinp) >= 1.0f) {
    pitch = (sinp >= 0.0f) ? HALF_PI : -HALF_PI;
  } else {
    pitch = asin(sinp);
  }

  float siny = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  float yaw = atan2(siny, cosy);

  return new PVector(degrees(roll), degrees(pitch), degrees(yaw));
}

void drawHUD() {
  hint(DISABLE_DEPTH_TEST);
  camera();
  noLights();

  fill(20);
  textAlign(LEFT, TOP);
  textSize(28);
  text("Compare orientation by 3D Model", 20, 18);

  textSize(14);
  text("Port: " + selectedPort + "   Baud: " + BAUDRATE + "   [C] reconnect  [R] reset", 20, 50);
  text("BMI160 lines: " + bmiCount + "   Multi-IMU lines: " + mimuCount, 20, 72);

  fill(20);
  textAlign(LEFT, TOP);
  textSize(64);
  text(formatElapsedHms(millis() - elapsedBaseMillis), width - 300, 28);

  textSize(18);

  fill(255, 145, 60);
  rect(20, height - 104, 16, 16);
  fill(30);
  text("BMI160 model", 44, height - 105);

  fill(70, 170, 255);
  rect(170, height - 104, 16, 16);
  fill(30);
  text("Multi-IMU model", 194, height - 105);

  PVector bEuler = quatToEulerDeg(bmiOrientation);
  PVector mEuler = quatToEulerDeg(mimuOrientation);

  String bQuat = "BMI160    q=[" + nf(bmiOrientation.w, 1, 5) + ", " + nf(bmiOrientation.x, 1, 5) + ", " + nf(bmiOrientation.y, 1, 5) + ", " + nf(bmiOrientation.z, 1, 5) + "]";
  String mQuat = "Multi-IMU q=[" + nf(mimuOrientation.w, 1, 5) + ", " + nf(mimuOrientation.x, 1, 5) + ", " + nf(mimuOrientation.y, 1, 5) + ", " + nf(mimuOrientation.z, 1, 5) + "]";

  String bEulerText = "BMI160    e=[" + nf(bEuler.x, 1, 2) + ", " + nf(bEuler.y, 1, 2) + ", " + nf(bEuler.z, 1, 2) + "]";
  String mEulerText = "Multi-IMU e=[" + nf(mEuler.x, 1, 2) + ", " + nf(mEuler.y, 1, 2) + ", " + nf(mEuler.z, 1, 2) + "]";

  int leftDataX = 420;
  int rightDataX = 920;
  fill(20);
  text(bQuat, leftDataX, height - 105);
  text(mQuat, leftDataX, height - 83);
  text(bEulerText, rightDataX, height - 105);
  text(mEulerText, rightDataX, height - 83);

  hint(ENABLE_DEPTH_TEST);
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
    statusText = "Connected to " + ports[idx];
  } catch (Exception e) {
    statusText = "Connect failed: " + e.getMessage();
  }
}

int findPortIndex(String[] ports, String preferred) {
  for (int i = 0; i < ports.length; i++) {
    if (ports[i].equalsIgnoreCase(preferred)) {
      return i;
    }
  }
  return -1;
}

String formatElapsedHms(int elapsedMs) {
  int totalSec = max(0, elapsedMs / 1000);
  int hours = totalSec / 3600;
  int minutes = (totalSec % 3600) / 60;
  int seconds = totalSec % 60;
  return nf(hours, 2) + ":" + nf(minutes, 2) + ":" + nf(seconds, 2);
}