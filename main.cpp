// printing sensor data with address

/*
 * Raw sensor printer - address + value only, slow print rate.
 *
 * No HID output, no fusion math, no demo/auto-rotate logic - just
 * reads all three MLX90393s and prints "[addr] x= y= z=" for each,
 * a few times a second instead of ~50Hz, so it's actually readable
 * in Serial Monitor while debugging wiring (e.g. the 0xD/0xE dead
 * sensor issue - watch for any address stuck at exactly 0,0,0).
 */

#include <Arduino.h>
#include <Adafruit_MLX90393.h>

#define ADDR_MAG1 0x0C
#define ADDR_MAG2 0x0D
#define ADDR_MAG3 0x0E

Adafruit_MLX90393 mag1 = Adafruit_MLX90393();
Adafruit_MLX90393 mag2 = Adafruit_MLX90393();
Adafruit_MLX90393 mag3 = Adafruit_MLX90393();

// How often to print - much slower than the ~50Hz control-loop rate,
// since this is for reading by eye, not for feeding a filter.
const unsigned long PRINT_INTERVAL_MS = 100;  // prints/sec

unsigned long lastPrint = 0;

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

void printOne(uint8_t addr, bool ok, float x, float y, float z) {
  Serial.print("[0x"); Serial.print(addr, HEX); Serial.print("] ");
  if (!ok) {
    Serial.println("read failed");
    return;
  }
  Serial.print("x="); Serial.print(x, 1);
  Serial.print("  y="); Serial.print(y, 1);
  Serial.print("  z="); Serial.println(z, 1);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();

  bool ok1 = initSensor(mag1, ADDR_MAG1);
  bool ok2 = initSensor(mag2, ADDR_MAG2);
  bool ok3 = initSensor(mag3, ADDR_MAG3);

  Serial.println("Sensor init:");
  Serial.print("  0x0C: "); Serial.println(ok1 ? "OK" : "FAILED");
  Serial.print("  0x0D: "); Serial.println(ok2 ? "OK" : "FAILED");
  Serial.print("  0x0E: "); Serial.println(ok3 ? "OK" : "FAILED");
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = now;

  float x1, y1, z1, x2, y2, z2, x3, y3, z3;
  bool ok1 = mag1.readData(&x1, &y1, &z1);
  bool ok2 = mag2.readData(&x2, &y2, &z2);
  bool ok3 = mag3.readData(&x3, &y3, &z3);

  printOne(ADDR_MAG1, ok1, x1, y1, z1);
  printOne(ADDR_MAG2, ok2, x2, y2, z2);
  printOne(ADDR_MAG3, ok3, x3, y3, z3);
  Serial.println();  // blank line between reading groups for readability
}