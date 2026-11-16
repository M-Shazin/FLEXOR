#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// HARDWARE PIN DEFINITIONS (Ideaspark ESP32 0.96" OLED Board)
// =============================================================================
#define I2C_SDA_PIN       21   // Hardware I2C SDA (Physical Pin 11)
#define I2C_SCL_PIN       22   // Hardware I2C SCL (Physical Pin 14)
#define I2C_FREQUENCY     400000 // 400kHz Fast Mode I2C

// =============================================================================
// I2C ADDRESSES
// =============================================================================
#define OLED_I2C_ADDRESS  0x3C // Standard built-in SSD1306 OLED address
#define MPU_I2C_ADDRESS   0x68 // MPU6050 default address (AD0 to GND)

// =============================================================================
// OLED DISPLAY CONFIGURATION
// =============================================================================
#define SCREEN_WIDTH      128  // OLED display width in pixels
#define SCREEN_HEIGHT     64   // OLED display height in pixels
#define OLED_RESET_PIN    -1   // Reset pin (-1 if sharing Arduino reset pin)
#define OLED_ROTATION     2    // 0 = standard, 2 = 180-degree flip (upside-down fix)

// =============================================================================
// LEG KINEMATICS & STATE THRESHOLDS
// =============================================================================
// Biomechanical state thresholds (in degrees):
//   - When lower leg is extended/straight (hanging down): Angle is small -> OPEN
//   - When lower leg moves up/flexes (knee bent): Angle increases -> CLOSED
#define THRESHOLD_LEG_OPEN_DEG    30.0f  // Below this angle: state is definitely OPEN
#define THRESHOLD_LEG_CLOSED_DEG  55.0f  // Above this angle: state is definitely CLOSED
                                         // Between 30° and 55°: Hysteresis holds previous state

// Angle Calibration Offset (Degrees):
// Adjust this if your sensor at rest doesn't read 0°.
#define ANGLE_ZERO_OFFSET_DEG     0.0f

// Filter coefficient (0.0 to 1.0)
// Higher alpha (>0.90) trusts gyro for fast movement, blends accel to eliminate drift.
#define COMPLEMENTARY_ALPHA       0.96f

// =============================================================================
// UPDATE RATES & TIMING (Non-blocking)
// =============================================================================
#define SENSOR_SAMPLE_INTERVAL_MS 20  // 50 Hz sensor reading
#define DISPLAY_REFRESH_INTERVAL_MS 50 // 20 Hz flicker-free display refresh
#define SERIAL_TELEMETRY_INTERVAL_MS 100 // 10 Hz Serial debug output

#endif // CONFIG_H
