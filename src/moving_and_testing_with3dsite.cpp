#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Adafruit_MLX90393.h>
#include <math.h>

// ---------------------------------------------------------------------
// USB HID: standard Gamepad descriptor (X,Y,Z,Rx,Ry,Rz + buttons).
// Generic HID class - no vendor driver needed, testable in any browser.
// ---------------------------------------------------------------------
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};

Adafruit_USBD_HID usb_hid;

// ---------------------------------------------------------------------
// Sensor addresses -> physical position (from your soldered jumpers)
// ---------------------------------------------------------------------
#define ADDR_MAG1 0x0C  // bottom
#define ADDR_MAG2 0x0D  // top left
#define ADDR_MAG3 0x0E  // top right

Adafruit_MLX90393 mag1 = Adafruit_MLX90393();
Adafruit_MLX90393 mag2 = Adafruit_MLX90393();
Adafruit_MLX90393 mag3 = Adafruit_MLX90393();

float off1x, off1y, off1z;
float off2x, off2y, off2z;
float off3x, off3y, off3z;

// ---------------------------------------------------------------------
// Tuning constants (ported from the Thonny test script / Config.h)
// ---------------------------------------------------------------------
const float GAIN_T[3]   = {28.0, 28.0, 24.0};   // Tx, Ty, Tz
const float GAIN_R[3]   = {18.0, 18.0, 20.0};   // Rx, Ry, Rz
const int   SIGN_AXIS[6] = {-1, 1, -1, 1, 1, 1};
const float DEAD_T = 16.0;
const float DEAD_R = 20.0;
const float SMOOTH_TAU_S = 0.08;

// Multiplies the normalized axis value up into the int8 report range.
// Tune this once you see real numbers on the tester - too low and the
// tester barely moves, too high and it clips at max deflection early.
const float OUTPUT_SCALE = 25.0;

// Sensor positions (unit circle, 120 deg apart) - only used for Rz
const float POS[3][2] = {
  {0.0f, -1.0f},                          // mag1 - bottom
  {-0.8660254f, 0.5f},                    // mag2 - top left
  {0.8660254f, 0.5f},                     // mag3 - top right
};

float smoothed[6] = {0, 0, 0, 0, 0, 0};
unsigned long last_t = 0;

float dead_zone(float value, float threshold) {
  if (fabs(value) < threshold) return 0.0f;
  return value > 0 ? value - threshold : value + threshold;
}

int8_t clamp_to_int8(float v) {
  if (v > 127) v = 127;
  if (v < -127) v = -127;
  return (int8_t)v;
}

// Matches mlx90393_triple_test_thonny.py's print format:
//   [0xc] x=  12.3 y=  -4.1 z=  88.0   [0xd] x=...  [0xe] x=...
void printRawSensors(float x1, float y1, float z1,
                      float x2, float y2, float z2,
                      float x3, float y3, float z3) {
  Serial.print("[0x"); Serial.print(ADDR_MAG1, HEX); Serial.print("] x=");
  Serial.print(x1, 1); Serial.print(" y="); Serial.print(y1, 1);
  Serial.print(" z="); Serial.print(z1, 1); Serial.print("   ");

  Serial.print("[0x"); Serial.print(ADDR_MAG2, HEX); Serial.print("] x=");
  Serial.print(x2, 1); Serial.print(" y="); Serial.print(y2, 1);
  Serial.print(" z="); Serial.print(z2, 1); Serial.print("   ");

  Serial.print("[0x"); Serial.print(ADDR_MAG3, HEX); Serial.print("] x=");
  Serial.print(x3, 1); Serial.print(" y="); Serial.print(y3, 1);
  Serial.print(" z="); Serial.println(z3, 1);
}

// Matches spacemouse_6dof_test.py's print format:
//   Tx=  0.12 Ty= -0.03 Tz=  0.00 Rx=  0.00 Ry=  0.00 Rz=  0.01
void printFusedAxes(float Tx, float Ty, float Tz, float Rx, float Ry, float Rz) {
  Serial.print("  Tx="); Serial.print(Tx, 2);
  Serial.print(" Ty=");  Serial.print(Ty, 2);
  Serial.print(" Tz=");  Serial.print(Tz, 2);
  Serial.print(" Rx=");  Serial.print(Rx, 2);
  Serial.print(" Ry=");  Serial.print(Ry, 2);
  Serial.print(" Rz=");  Serial.println(Rz, 2);
}

bool initSensor(Adafruit_MLX90393 &s, uint8_t addr) {
  if (!s.begin_I2C(addr, &Wire)) return false;
  s.setGain(MLX90393_GAIN_1X);
  s.setResolution(MLX90393_X, MLX90393_RES_16);
  s.setResolution(MLX90393_Y, MLX90393_RES_16);
  s.setResolution(MLX90393_Z, MLX90393_RES_16);
  s.setOversampling(MLX90393_OSR_3);
  s.setFilter(MLX90393_FILTER_5);
  return true;
}

void setup() {
  Serial.begin(115200);

  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  Wire.begin();

  bool ok1 = initSensor(mag1, ADDR_MAG1);
  bool ok2 = initSensor(mag2, ADDR_MAG2);
  bool ok3 = initSensor(mag3, ADDR_MAG3);

  if (!ok1 || !ok2 || !ok3) {
    Serial.println("One or more sensors failed to init - check wiring/addresses.");
  }

  // Calibrate: keep the knob still during boot
  delay(200);
  mag1.readData(&off1x, &off1y, &off1z);
  mag2.readData(&off2x, &off2y, &off2z);
  mag3.readData(&off3x, &off3y, &off3z);

  last_t = millis();
}

void loop() {
  float x1, y1, z1, x2, y2, z2, x3, y3, z3;
  mag1.readData(&x1, &y1, &z1);
  mag2.readData(&x2, &y2, &z2);
  mag3.readData(&x3, &y3, &z3);

  float m1x = x1 - off1x, m1y = y1 - off1y, m1z = z1 - off1z;
  float m2x = x2 - off2x, m2y = y2 - off2y, m2z = z2 - off2z;
  float m3x = x3 - off3x, m3y = y3 - off3y, m3z = z3 - off3z;

  // --- Raw 6DoF estimate (cad-mouse-mk2 formulas) ---
  float Tx = (m1x + m2x + m3x) / 3.0f;
  float Ty = (m1y + m2y + m3y) / 3.0f;
  float Tz = (m1z + m2z + m3z) / 3.0f;

  float Rx = sqrtf(3.0f) * (m2z + m3z - 2 * m1z) / 3.0f;
  float Ry = m3z - m2z;

  float Rz = 0.0f;
  Rz += POS[0][0] * m1y - POS[0][1] * m1x;
  Rz += POS[1][0] * m2y - POS[1][1] * m2x;
  Rz += POS[2][0] * m3y - POS[2][1] * m3x;

  float axes[6]  = {Tx, Ty, Tz, Rx, Ry, Rz};
  float dead[6]  = {DEAD_T, DEAD_T, DEAD_T, DEAD_R, DEAD_R, DEAD_R};
  float gain[6]  = {GAIN_T[0], GAIN_T[1], GAIN_T[2], GAIN_R[0], GAIN_R[1], GAIN_R[2]};

  unsigned long now = millis();
  float dt = (now - last_t) / 1000.0f;
  last_t = now;
  float alpha = dt / (SMOOTH_TAU_S + dt);

  for (int i = 0; i < 6; i++) {
    float v = dead_zone(axes[i], dead[i]);
    v = v / gain[i] * SIGN_AXIS[i];
    smoothed[i] += alpha * (v - smoothed[i]);
  }

  hid_gamepad_report_t report;
  report.x  = clamp_to_int8(smoothed[0] * OUTPUT_SCALE);
  report.y  = clamp_to_int8(smoothed[1] * OUTPUT_SCALE);
  report.z  = clamp_to_int8(smoothed[2] * OUTPUT_SCALE);
  report.rx = clamp_to_int8(smoothed[3] * OUTPUT_SCALE);
  report.ry = clamp_to_int8(smoothed[4] * OUTPUT_SCALE);
  report.rz = clamp_to_int8(smoothed[5] * OUTPUT_SCALE);
  report.hat = 0;
  // TODO: wire the 2 side buttons to GPIO pins and read them here.
  report.buttons = 0;

  if (usb_hid.ready()) {
    usb_hid.sendReport(0, &report, sizeof(report));
  }

  printRawSensors(x1, y1, z1, x2, y2, z2, x3, y3, z3);
  printFusedAxes(Tx, Ty, Tz, Rx, Ry, Rz);

  delay(20); // ~50Hz report + print rate
}