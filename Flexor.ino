/**
 * @file Flexor.ino
 * @brief Real-Time Leg Extension/Flexion Monitor using ESP32 & MPU6050
 * 
 * Hardware:
 *   - ideaspark ESP32 0.96" OLED Board (ESP32-WROOM-32)
 *   - Built-in 0.96" I2C OLED (SSD1306, 128x64) on SDA: GPIO 21, SCL: GPIO 22
 *   - MPU6050 6-DoF Accelerometer & Gyroscope on SDA: GPIO 21, SCL: GPIO 22
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "config.h"

// Hardware instances
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
Adafruit_MPU6050 mpu;

// Leg State Definition
enum LegState {
  LEG_CALIBRATING,
  LEG_OPEN,    // Leg extended / straight
  LEG_CLOSED   // Leg bent / flexed
};

LegState currentLegState = LEG_CALIBRATING;

// Kinematics and Filter variables
float rawPitchAngle     = 0.0f;
float filteredAngle     = 0.0f;
float zeroOffsetAngle   = ANGLE_ZERO_OFFSET_DEG;
float gyroRateDegPerSec = 0.0f;

// Loop timing
unsigned long lastSensorSampleTime = 0;
unsigned long lastDisplayTime      = 0;
unsigned long lastSerialTime       = 0;
unsigned long lastFilterMicros     = 0;

// Device status flags
bool oledReady = false;
bool mpuReady  = false;

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void drawSplashScreen();
void drawErrorScreen(const char* message);
void updateDisplay();
void processSerialCommands();
float calculatePitchAngle(float ax, float ay, float az);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("==================================================");
  Serial.println("   FLEXOR: Real-Time Leg Flexion/Extension Monitor ");
  Serial.println("   Board: ideaspark ESP32 0.96\" OLED + MPU6050    ");
  Serial.println("==================================================");

  // Initialize I2C Bus on GPIO 21 (SDA) and GPIO 22 (SCL)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);

  // 1. Initialize OLED Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println("[ERROR] Built-in SSD1306 OLED not detected at 0x3C!");
  } else {
    oledReady = true;
    display.setRotation(OLED_ROTATION);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    drawSplashScreen();
    Serial.println("[OK] Built-in OLED initialized (128x64 @ 0x3C).");
  }

  // 2. Initialize MPU6050 Sensor
  if (!mpu.begin(MPU_I2C_ADDRESS, &Wire)) {
    Serial.println("[ERROR] MPU6050 not detected at 0x68!");
    Serial.println("        Please check VCC->3V3, GND->GND, SDA->D21, SCL->D22.");
    if (oledReady) {
      drawErrorScreen("MPU6050 NOT FOUND\nCheck SDA:21 SCL:22");
    }
  } else {
    mpuReady = true;
    Serial.println("[OK] MPU6050 detected and initialized @ 0x68.");

    // Configure MPU6050 ranges and digital low-pass filter
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Filters out high-frequency vibrations

    // Prime the complementary filter with initial reading
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    rawPitchAngle = calculatePitchAngle(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    filteredAngle = rawPitchAngle;
    lastFilterMicros = micros();

    currentLegState = (filteredAngle < THRESHOLD_LEG_OPEN_DEG) ? LEG_OPEN : LEG_CLOSED;
  }

  delay(1200); // Allow user to see splash screen
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // Allow user to send 'z' in Serial Monitor to Tare/Zero current angle
  processSerialCommands();

  // --- SENSOR ACQUISITION & FILTERING (50 Hz / 20 ms) ---
  if (mpuReady && (currentMillis - lastSensorSampleTime >= SENSOR_SAMPLE_INTERVAL_MS)) {
    lastSensorSampleTime = currentMillis;

    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastFilterMicros) / 1000000.0f;
    lastFilterMicros = currentMicros;
    if (dt <= 0.0f || dt > 0.1f) dt = (float)SENSOR_SAMPLE_INTERVAL_MS / 1000.0f;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate raw pitch tilt from gravity vector
    rawPitchAngle = calculatePitchAngle(a.acceleration.x, a.acceleration.y, a.acceleration.z);

    // Gyro angular rate around Y axis (in degrees/sec)
    // Conversion: rad/s * (180 / PI)
    gyroRateDegPerSec = g.gyro.y * 57.2957795f;

    // Complementary Filter: Blends fast gyro integration with stable gravity vector
    filteredAngle = (COMPLEMENTARY_ALPHA * (filteredAngle + gyroRateDegPerSec * dt)) + 
                    ((1.0f - COMPLEMENTARY_ALPHA) * rawPitchAngle);

    // Subtract zero calibration offset
    float activeAngle = filteredAngle - zeroOffsetAngle;

    // Absolute flexion magnitude for leg extension/flexion
    float absAngle = fabs(activeAngle);

    // State Classification with Hysteresis
    if (absAngle < THRESHOLD_LEG_OPEN_DEG) {
      currentLegState = LEG_OPEN;
    } else if (absAngle > THRESHOLD_LEG_CLOSED_DEG) {
      currentLegState = LEG_CLOSED;
    }
    // Note: If between 30° and 55°, keep currentLegState (prevents chattering)
  }

  // --- DISPLAY REFRESH (20 Hz / 50 ms) ---
  if (oledReady && (currentMillis - lastDisplayTime >= DISPLAY_REFRESH_INTERVAL_MS)) {
    lastDisplayTime = currentMillis;
    updateDisplay();
  }

  // --- SERIAL TELEMETRY (10 Hz / 100 ms) ---
  if (currentMillis - lastSerialTime >= SERIAL_TELEMETRY_INTERVAL_MS) {
    lastSerialTime = currentMillis;
    float currentRelAngle = filteredAngle - zeroOffsetAngle;

    Serial.print("Angle:");
    Serial.print(currentRelAngle, 1);
    Serial.print(" deg | State:");
    if (currentLegState == LEG_OPEN) {
      Serial.print("OPEN");
    } else if (currentLegState == LEG_CLOSED) {
      Serial.print("CLOSED");
    } else {
      Serial.print("INIT");
    }
    Serial.print(" | RawTilt:");
    Serial.print(rawPitchAngle, 1);
    Serial.println();
  }
}

// =============================================================================
// KINEMATICS: PITCH / TILT CALCULATION
// =============================================================================
/**
 * Calculates tilt angle (pitch) in degrees using the gravity acceleration vector.
 * Works seamlessly whether the shin moves forward or backward.
 */
float calculatePitchAngle(float ax, float ay, float az) {
  // Sagittal plane pitch angle using atan2
  float denominator = sqrt((ay * ay) + (az * az));
  if (denominator < 0.001f) denominator = 0.001f;
  
  float pitchRad = atan2(-ax, denominator);
  return pitchRad * 57.2957795f; // Convert radians to degrees
}

// =============================================================================
// OLED DISPLAY RENDERING (Flicker-Free Off-Screen Buffer)
// =============================================================================
void updateDisplay() {
  display.clearDisplay();

  // 1. Top Inverted Header Bar
  display.fillRect(0, 0, SCREEN_WIDTH, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(14, 2);
  display.print("FLEXOR: LEG MONITOR");

  // Restore white text on black background
  display.setTextColor(SSD1306_WHITE);

  if (!mpuReady) {
    display.setCursor(4, 24);
    display.setTextSize(1);
    display.println("MPU6050 Disconnected!");
    display.setCursor(4, 40);
    display.println("Check SDA/SCL wires");
    display.display();
    return;
  }

  // 2. Large Central State Banner (Size 2)
  display.setTextSize(2);
  if (currentLegState == LEG_OPEN) {
    // Center ">> OPEN <<"
    display.setCursor(10, 16);
    display.print(">> OPEN <<");
  } else {
    // Center "[ CLOSED ]"
    display.setCursor(6, 16);
    display.print("[ CLOSED ]");
  }

  // 3. Flexion Visual Gauge (Dynamic Progress Bar)
  float relAngle = fabs(filteredAngle - zeroOffsetAngle);
  // Map angle (0° to 90°) to bar width (0 to 118 px)
  int barWidth = constrain((int)(relAngle * (118.0f / 90.0f)), 0, 118);

  display.drawRect(4, 35, 120, 9, SSD1306_WHITE);
  if (barWidth > 0) {
    display.fillRect(6, 37, barWidth, 5, SSD1306_WHITE);
  }

  // 4. Bottom Numerical Readout & Sub-status
  display.setTextSize(1);
  display.setCursor(4, 50);
  display.print("Angle:");
  display.print(relAngle, 1);
  display.print((char)247); // Degree symbol

  display.setCursor(84, 50);
  if (currentLegState == LEG_OPEN) {
    display.print("[STRAIGHT]");
  } else {
    display.print(" [FLEXED] ");
  }

  // Render complete buffer to screen
  display.display();
}

// =============================================================================
// SPLASH & ERROR SCREENS
// =============================================================================
void drawSplashScreen() {
  display.clearDisplay();
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 12);
  display.println("F L E X O R");
  display.setCursor(8, 26);
  display.println("Leg Motion Tracker");
  display.setCursor(14, 42);
  display.println("MPU6050 + ESP32");
  display.display();
}

void drawErrorScreen(const char* message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 4);
  display.println("! SYSTEM ERROR !");
  display.drawLine(0, 14, SCREEN_WIDTH, 14, SSD1306_WHITE);
  display.setCursor(0, 22);
  display.println(message);
  display.display();
}

// =============================================================================
// SERIAL INTERFACE (Calibration Commands)
// =============================================================================
void processSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'z' || cmd == 'Z' || cmd == '0') {
      zeroOffsetAngle = filteredAngle;
      Serial.println();
      Serial.print("[CALIBRATION] Zero offset calibrated to: ");
      Serial.print(zeroOffsetAngle, 2);
      Serial.println(" deg");
    }
  }
}
