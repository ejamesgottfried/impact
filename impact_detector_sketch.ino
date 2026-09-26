// This is a simple impact detector using an IMU sensor: MPU 6050
// Watches for total acceleration crossing a threshold, then buzzes and shows an alert. 
// Also detects when the sensor has frozen 
// (reads 0.00 repeatedly, a glitch that often occured after hard impact) and automatically reinitializes itself.

// IMU Wiring:
// GND -> GND (Ground, Zero volt reference)
// VCC -> 3.3V (Power, be sure not to use 5V, as this could damage the chip)
// SCL -> A5 (Serial Clock, used in I2C (Inter-Integrated Circuit) communication to synchronize data sent across the serial data line)
// SDA -> A4 (Signal line used in the I2C communication protocol.)

// Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// IMU
Adafruit_MPU6050 mpu;

// LCD: wire to RS, E, D4, D5, D6, D7 respectively
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Buzzer
const int BUZZER_PIN = 8;

// Settings you can tune
const int BEEP_DURATION_MS = 100;
const float IMPACT_THRESHOLD = 3.5;          // accelation in g's, magnitude above this counts as a hard impact
const unsigned long SAMPLE_INTERVAL_MS = 20; // 50 samples per second
const unsigned long ALERT_DISPLAY_MS = 2000; // how long the alert stays on screen after impact

// Frozen sensor detection
const int ZERO_STUCK_LIMIT = 5; // how many 0.00 readings in a row before we consider it frozen
int zeroCount = 0;

// State
bool alertActive = false;
unsigned long alertStartedAt = 0;

unsigned long lastSampleAt = 0;

// Initialize IMU sensor
void initSensor() {
  Serial.println("Initializing MPU6050...");
  Wire.end();
  delay(100);
  Wire.begin();
  delay(100);

  // Display if error
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 ready.");
}

// Setup runs once, when powered on or reset, to initialize sensor, LCD, and buzzer and establish communication speed
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);

  initSensor();

  lcd.begin(16, 2);
  lcd.print("Monitoring...");
}

// Loop runs continuously, reading data from the IMU sensor
void loop() {
  unsigned long now = millis();
  if (now - lastSampleAt < SAMPLE_INTERVAL_MS) return;
  lastSampleAt = now;

  // If an alert is showing, just wait for it to time out, then reset
  if (alertActive) {
    if (now - alertStartedAt > ALERT_DISPLAY_MS) {
      alertActive = false;
      lcd.clear();
      lcd.print("Monitoring...");
    }
    return;
  }

  // Read the sensor and get total acceleration magnitude
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax_g = a.acceleration.x / 9.81;
  float ay_g = a.acceleration.y / 9.81;
  float az_g = a.acceleration.z / 9.81;
  float magnitude = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  // Check if the sensor has frozen
  // A resting reading should be close to 1.00g (gravity).
  // Repeated exact 0.00 readings means the I2C connection glitched (common after impact) and the sensor needs to be reinitialized.
  if (magnitude == 0.00) {
    zeroCount++;
    if (zeroCount >= ZERO_STUCK_LIMIT) {
      Serial.println("Sensor appears frozen, reinitializing...");
      initSensor();
      zeroCount = 0;
      return;
    }
  } else {
    zeroCount = 0;
  }

  // Print anything getting close to or crossing the threshold, useful for tuning
  const float PRINT_MARGIN = 0.5;
  if (magnitude > IMPACT_THRESHOLD - PRINT_MARGIN) {
    Serial.print("magnitude: ");
    Serial.print(magnitude);
    if (magnitude > IMPACT_THRESHOLD) {
      Serial.println("  <-- ABOVE THRESHOLD");
    } else {
      Serial.println("  (approaching threshold)");
    }
  }

  // Check if a reading is a hard hit
  if (magnitude > IMPACT_THRESHOLD) {
    triggerAlert();
  }
}

// Helper function: what to do when there is an impact
void triggerAlert() {
  alertActive = true;
  alertStartedAt = millis();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IMPACT DETECTED");

  tone(BUZZER_PIN, 2000, BEEP_DURATION_MS);
  delay(BEEP_DURATION_MS + 100);
  tone(BUZZER_PIN, 2000, BEEP_DURATION_MS);

  Serial.println("IMPACT DETECTED");
}