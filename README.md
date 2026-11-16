# Flexor: Real-Time Leg Flexion & Extension Tracker

Real-time biomechanical leg joint tracker using an **MPU6050 6-DoF IMU** mounted on the lower leg (shin) and an **ideaspark ESP32 0.96" OLED Board**.

---

## 1. Hardware Overview & Wiring

The **ideaspark ESP32 0.96" OLED Board** features an onboard SSD1306 OLED wired internally to **GPIO 21 (SDA)** and **GPIO 22 (SCL)**. Because I2C supports multiple devices on the same lines, the MPU6050 connects directly in parallel to the same pins.

### Wiring Diagram (Right Header of the Board)

All 4 connections are made to the **right-hand pin header**:

```
        Ideaspark ESP32 Board                 MPU6050 Module
      +------------------------+             +--------------+
      |                        |             |              |
      |   [Pin 1] 3.3V  -------+------------>| VCC          |
      |   [Pin 2] GND   -------+------------>| GND          |
      |   [Pin 11] D21  -------+------------>| SDA          |
      |   [Pin 14] D22  -------+------------>| SCL          |
      |                        |             | AD0 -> GND   |
      +------------------------+             +--------------+
```

| MPU6050 Pin | Ideaspark Pin | Board Pin Label | Description |
| :--- | :--- | :--- | :--- |
| **VCC** | Physical Pin 1 | **3V3** | 3.3V Power |
| **GND** | Physical Pin 2 | **GND** | Ground |
| **SDA** | Physical Pin 11 | **D21** | I2C Data (shared with OLED) |
| **SCL** | Physical Pin 14 | **D22** | I2C Clock (shared with OLED) |
| **AD0** | Connect to GND | **GND** | Sets I2C address to `0x68` |

---

## 2. Mounting the MPU6050 on the Leg

1. Attach the **MPU6050** securely to the **lower leg (shin/shank)** of the model using double-sided tape, zip ties, or an elastic strap.
2. Ensure the sensor is rigid relative to the shin so it does not wobble independently.
3. When the leg is hanging straight down (extended), the system will register **`LEG: OPEN`**.
4. When the leg flexes (knee bends) upward or backward, the tilt angle increases. When it crosses the threshold (default $> 55^\circ$), the display immediately switches to **`LEG: CLOSED`**.

---

## 3. How to Upload

### Method A: Arduino IDE (GUI)
1. Open **Arduino IDE**.
2. Go to **File -> Open...** and open `Flexor/Flexor.ino`.
3. Under **Tools -> Board**, select:
   - **ESP32 Arduino** $\rightarrow$ **DOIT ESP32 DEVKIT V1** (or **ESP32 Dev Module**).
4. Under **Tools -> Port**, select your serial port (e.g. `/dev/cu.usbserial-10`).
5. Ensure the required libraries are installed (via **Sketch -> Include Library -> Manage Libraries...**):
   - `Adafruit SSD1306`
   - `Adafruit GFX Library`
   - `Adafruit MPU6050`
   - `Adafruit BusIO`
   - `Adafruit Unified Sensor`
6. Click **Upload** (arrow icon).

---

## 4. Calibration & Tuning (`config.h`)

You can customize parameters directly in `config.h`:
- `THRESHOLD_LEG_OPEN_DEG`: Angle (degrees) below which the leg is marked **OPEN** (default `30.0f`).
- `THRESHOLD_LEG_CLOSED_DEG`: Angle (degrees) above which the leg is marked **CLOSED** (default `55.0f`).
- `COMPLEMENTARY_ALPHA`: Complementary filter weight (default `0.96f`).

### Instant Zeroing (Tare) via Serial Monitor:
1. Open the Arduino **Serial Monitor** at **115200 baud**.
2. Hold the leg in the desired "straight / rest" position.
3. Type `z` and press **Enter**.
4. The system will immediately re-tare the reference angle to $0.0^\circ$.
