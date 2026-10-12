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
//
// Gains bumped ~4x from the original disc-magnet defaults - cuboid
// magnets push a much stronger raw field, so a bigger divisor is
// needed to stop a tiny nudge from slamming straight to max output.
// Retune by feel: raise further if it still maxes out on a small
// movement, lower if it now feels sluggish/unresponsive.
// ---------------------------------------------------------------------
const float GAIN_T[3]   = {280.0, 280.0, 265.0};  // Tx, Ty, Tz
const float GAIN_R[3]   = {240.0, 240.0, 250.0};  // Rx, Ry, Rz
// Rz flipped (was +1) - physical clockwise twist was coming out as
// counter-clockwise rotation in the visualizer. Pure sign fix - gains,
// calibration, and demo logic below are untouched from your working copy.
const int   SIGN_AXIS[6] = {-1, 1, -1, 1, 1, -1};
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

// ---------------------------------------------------------------------
// Auto-demo state machine
//
// Idle for IDLE_BEFORE_DEMO_MS with no real nudge -> starts cycling
// through each axis in turn (auto-generated motion, same HID path as
// real sensor data). The instant a real nudge is detected - based on
// the ACTUAL smoothed sensor output exceeding NUDGE_THRESHOLD, not a
// separate fake signal - the demo stops immediately and real sensor
// data takes over for HOLD_AFTER_NUDGE_MS before idle timing resets
// and demo mode can resume.
//
// This sits on top of the existing validated + filtered pipeline
// above, not instead of it - a nudge is only "real" if it survived
// the range/jump gating and smoothing, so demo mode won't falsely
// exit on a sensor glitch.
// ---------------------------------------------------------------------
enum DemoState { DEMO_IDLE_WAIT, DEMO_RUNNING, DEMO_PAUSED_FOR_HUMAN };
DemoState demoState = DEMO_IDLE_WAIT;

const unsigned long IDLE_BEFORE_DEMO_MS  = 5000;   // no nudge for this long -> start demo
const unsigned long HOLD_AFTER_NUDGE_MS  = 4000;   // stay paused this long after last nudge
const unsigned long DEMO_AXIS_HOLD_MS    = 2500;   // time spent per axis in the demo
const float NUDGE_THRESHOLD              = 0.15f;  // smoothed-axis units; above this = real input

unsigned long lastNudgeTime  = 0;
unsigned long demoAxisStart  = 0;
int demoAxisIndex            = 0;

// Demo cycles through all 6 axes in this order: Tx,Ty,Tz,Rx,Ry,Rz
const char *axisNames[6] = {"Tx", "Ty", "Tz", "Rx", "Ry", "Rz"};

// ---------------------------------------------------------------------
// Read validity gating - separate problem from gain saturation.
// "Jumping around" means individual reads are implausible (glitchy I2C
// transaction, sensor briefly out of range, one axis spiking while the
// others stay flat). Gain scaling can't fix that - it just amplifies
// or shrinks bad data along with good data. Instead we check each raw
// read against two things before trusting it:
//   1. MAX_FIELD_UT   - is the absolute reading physically plausible
//      for this sensor/magnet setup, or did the chip return garbage?
//   2. MAX_JUMP_UT     - did this read move an implausible amount since
//      last sample (one 20ms tick) given how fast a hand can move?
// A read that fails either check is thrown out and we hold the last
// good value instead of feeding the jump into the smoothing filter.
// Tune both from the raw Serial output: watch printRawSensors() while
// deliberately moving slowly vs. glitching, and set the thresholds
// just above normal motion, well below the glitch spikes.
// ---------------------------------------------------------------------
const float MAX_FIELD_UT = 3000.0f;   // ignore any axis reading beyond this
const float MAX_JUMP_UT  = 600.0f;    // ignore a single-sample jump beyond this

float last_good[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
bool  have_last_good   = false;

// Returns true if the read is plausible; on success also updates
// last_good so the next call's jump-check has something to compare to.
bool validateSensorRead(int idx, float x, float y, float z,
                         float &outx, float &outy, float &outz) {
  bool rangeOk = fabs(x) < MAX_FIELD_UT && fabs(y) < MAX_FIELD_UT && fabs(z) < MAX_FIELD_UT;

  bool jumpOk = true;
  if (have_last_good) {
    jumpOk = fabs(x - last_good[idx][0]) < MAX_JUMP_UT &&
             fabs(y - last_good[idx][1]) < MAX_JUMP_UT &&
             fabs(z - last_good[idx][2]) < MAX_JUMP_UT;
  }

  if (rangeOk && jumpOk) {
    last_good[idx][0] = x;
    last_good[idx][1] = y;
    last_good[idx][2] = z;
    outx = x; outy = y; outz = z;
    return true;
  }

  // Reject - hold the last known-good value for this sensor instead
  // of passing the glitch/jump through to the fusion math.
  outx = last_good[idx][0];
  outy = last_good[idx][1];
  outz = last_good[idx][2];
  return false;
}

float dead_zone(float value, float threshold) {
  if (fabs(value) < threshold) return 0.0f;
  return value > 0 ? value - threshold : value + threshold;
}

int8_t clamp_to_int8(float v) {
  if (v > 127) v = 127;
  if (v < -127) v = -127;
  return (int8_t)v;
}

// Produces a smooth sine ramp (0 -> peak -> 0) for whichever axis is
// currently active in the demo cycle, all other axes held at zero.
void demoAxisValues(unsigned long elapsedInAxis, float outAxes[6]) {
  for (int i = 0; i < 6; i++) outAxes[i] = 0.0f;

  float t = (float)elapsedInAxis / (float)DEMO_AXIS_HOLD_MS;
  if (t > 1.0f) t = 1.0f;
  float s = sinf(t * (float)M_PI);  // 0 -> 1 -> 0 across the hold window

  const float DEMO_PEAK = 1.0f;  // matches smoothed[] output scale (pre-OUTPUT_SCALE)
  outAxes[demoAxisIndex] = s * DEMO_PEAK;
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
  lastNudgeTime = millis();
}

void loop() {
  float raw_x1, raw_y1, raw_z1, raw_x2, raw_y2, raw_z2, raw_x3, raw_y3, raw_z3;
  bool ok1 = mag1.readData(&raw_x1, &raw_y1, &raw_z1);
  bool ok2 = mag2.readData(&raw_x2, &raw_y2, &raw_z2);
  bool ok3 = mag3.readData(&raw_x3, &raw_y3, &raw_z3);

  // Only trust reads that both (a) came back OK from the I2C
  // transaction and (b) pass the plausibility/jump check above.
  // Anything else falls back to the last known-good value for that
  // sensor, so a single glitchy sample can't spike the output.
  float x1, y1, z1, x2, y2, z2, x3, y3, z3;
  bool valid1 = ok1 && validateSensorRead(0, raw_x1, raw_y1, raw_z1, x1, y1, z1);
  bool valid2 = ok2 && validateSensorRead(1, raw_x2, raw_y2, raw_z2, x2, y2, z2);
  bool valid3 = ok3 && validateSensorRead(2, raw_x3, raw_y3, raw_z3, x3, y3, z3);
  if (!ok1) { x1 = last_good[0][0]; y1 = last_good[0][1]; z1 = last_good[0][2]; }
  if (!ok2) { x2 = last_good[1][0]; y2 = last_good[1][1]; z2 = last_good[1][2]; }
  if (!ok3) { x3 = last_good[2][0]; y3 = last_good[2][1]; z3 = last_good[2][2]; }
  have_last_good = true;

  if (!(valid1 && valid2 && valid3)) {
    Serial.print("rejected read - sensor(s) out of range/jumped: ");
    if (!valid1) Serial.print("1 ");
    if (!valid2) Serial.print("2 ");
    if (!valid3) Serial.print("3 ");
    Serial.println();
  }

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

  // --- Nudge detection: based on the REAL smoothed sensor output,
  // after gating and filtering, so a glitch or noise floor can't
  // fake a "human interfered" signal. ---
  bool realNudgeNow = false;
  for (int i = 0; i < 6; i++) {
    if (fabs(smoothed[i]) > NUDGE_THRESHOLD) { realNudgeNow = true; break; }
  }

  unsigned long now2 = millis();
  if (realNudgeNow) {
    lastNudgeTime = now2;
    if (demoState != DEMO_PAUSED_FOR_HUMAN) {
      Serial.println("Nudge detected - demo paused, showing real input.");
    }
    demoState = DEMO_PAUSED_FOR_HUMAN;
  }

  switch (demoState) {
    case DEMO_PAUSED_FOR_HUMAN:
      if (now2 - lastNudgeTime > HOLD_AFTER_NUDGE_MS) {
        demoState = DEMO_IDLE_WAIT;
        lastNudgeTime = now2;  // restart the idle countdown from here
      }
      break;

    case DEMO_IDLE_WAIT:
      if (now2 - lastNudgeTime > IDLE_BEFORE_DEMO_MS) {
        demoState = DEMO_RUNNING;
        demoAxisIndex = 0;
        demoAxisStart = now2;
        Serial.print("No input for a while - starting demo, axis: ");
        Serial.println(axisNames[demoAxisIndex]);
      }
      break;

    case DEMO_RUNNING:
      if (now2 - demoAxisStart > DEMO_AXIS_HOLD_MS) {
        demoAxisIndex = (demoAxisIndex + 1) % 6;
        demoAxisStart = now2;
        Serial.print("Demo moving to axis: ");
        Serial.println(axisNames[demoAxisIndex]);
      }
      break;
  }

  // --- Choose what actually goes out over HID this tick ---
  float outputAxes[6];
  if (demoState == DEMO_RUNNING) {
    demoAxisValues(now2 - demoAxisStart, outputAxes);
  } else {
    for (int i = 0; i < 6; i++) outputAxes[i] = smoothed[i];
  }

  hid_gamepad_report_t report;
  report.x  = clamp_to_int8(outputAxes[0] * OUTPUT_SCALE);
  report.y  = clamp_to_int8(outputAxes[1] * OUTPUT_SCALE);
  report.z  = clamp_to_int8(outputAxes[2] * OUTPUT_SCALE);
  report.rx = clamp_to_int8(outputAxes[3] * OUTPUT_SCALE);
  report.ry = clamp_to_int8(outputAxes[4] * OUTPUT_SCALE);
  report.rz = clamp_to_int8(outputAxes[5] * OUTPUT_SCALE);
  report.hat = 0;

  report.buttons = 0;

  if (usb_hid.ready()) {
    usb_hid.sendReport(0, &report, sizeof(report));
  }

  printRawSensors(x1, y1, z1, x2, y2, z2, x3, y3, z3);
  printFusedAxes(Tx, Ty, Tz, Rx, Ry, Rz);

  delay(20); // ~50Hz report + print rate
}