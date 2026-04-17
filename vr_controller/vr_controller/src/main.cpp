// --------------------------------------------------------------------------------------
// VR Controller Firmware - ESP32-S3
// Reads IMU data from MPU6050, runs Madgwick sensor fusion and transmits 
// ControllerPacket structs over WiFi UDP to the host PC driver.
// --------------------------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <MadgwickAHRS.h>
#include <WiFi.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Define pins for I2C communication
#define I2C_SDA 1
#define I2C_SCL 2

// --------------------------------------------------------------------------------------
// Configuration - edit these for network and setup
// --------------------------------------------------------------------------------------

// Your WiFi network credentials
const char* WIFI_SSID = "YOUR_NETWORK_NAME";
const char* WIFI_PASSWORD = "YOUR_NETWORK_PASSWORD";

// IP address of your host PC running the SteamVR driver
// Find this by running 'ipconfig' in a Windows command prompt and looking
// for the IPv4 Address under your active network adapter.
const char* HOST_PC_IP = "192.168.1.100"; // Replace with your host PC's IP Address

// UDP port the driver is listening on. Must match what's in device_provider.cpp.
// Left hand = 5555, Right hand = 5556.
// Change this per-controller by flashing different firmware to each ESP32,
// or use a physical switch/jumper to select at boot (shown below)
const uint16_t CONTROLLER_PORT = 5555;

// 0 = Left hand, 1 = Right hand. Must match the port above.
const uint8_t CONTROLLER_ID = 0;

// Sample rate in Hz. The Madgwick filter must be initialized with this value.
// 100 Hz is a good starting point; you can try up to 200 Hz with the MPU6050.
const int SAMPLE_RATE_HZ = 100;
const unsigned long SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_RATE_HZ; // microseconds

// --------------------------------------------------------------------------------------
// Packet definition - must be byte-for-byte identical to controller_device.h
// __attribute__((packed)) is the GCC/Clang equivalent of MSVC's #pragma pack,
// and tells the compiler not to insert any padding between struct fields.
// --------------------------------------------------------------------------------------
struct __attribute__((packed)) ControllerPacket {
  uint8_t header;         // Always 0xAA
  uint8_t controller_id;  // 0 = Left hand, 1 = Right hand
  float qw, qx, qy, qz;   // Orientation quaternion from Madgwick filter
  float ax, ay, az;       // Linear acceleration in m/s^2 (after gravity removal)
  float x, y, z;          // Position in meters - filled in by base stations, for now set to 0
  uint16_t checksum;      // Sum of all preceding bytes
};

struct GyroOffset { float x, y, z;};

// --------------------------------------------------------------------------------------
// Global variables
// --------------------------------------------------------------------------------------
TwoWire I2CIMU = TwoWire(0);
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
Madgwick filter;
WiFiUDP udp;


// Pre-resolved destaination address, set up in connectWiFi()
IPAddress host_id;

// Timing - we use micros() for high resolution sample rate control
unsigned long last_sample_time_us = 0;

// --------------------------------------------------------------------------------------
// Checksum - sums all bytes in the packet except the checksum field itself.
// Both the ESP32 and the driver use this same algorithm.
// --------------------------------------------------------------------------------------
uint16_t ComputeChecksum(const ControllerPacket& pkt) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&pkt);
  uint16_t sum = 0;
  // Subtract 2 to skip the checksum field at the end of the struct
  for (size_t i = 0; i < sizeof(ControllerPacket) - sizeof(uint16_t); ++i){
    sum += bytes[i];
  }
  return sum;
}

// --------------------------------------------------------------------------------------
// WiFi connection helper
// --------------------------------------------------------------------------------------
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);  // Station mode - connect to existing router
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait for connection, print a dot every 500ms
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nConnected! ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Parse the host PC IP string once into an IPAddress object.
  // Doing this once here is faster than parsing it every loop iteration.
  host_id.fromString(HOST_PC_IP);

  // Open the UDP socket on any local port (0 = let the OS choose)
  udp.begin(0);
}

// --------------------------------------------------------------------------------------
// IMU calibration - removes DC offset from gyroscope readings.
// The MPU6050 has nonzero gyro output even when completely stationary.
// This function averages ~500 samples at rest to find that offset.
// Call this once at startup with the controller sitting still on a flat surface.
// --------------------------------------------------------------------------------------
GyroOffset CalibrateGyro() {
  Serial.println("Calibrating gyroscope... Keep the controller still on a flat surface.");
  const int NUM_SAMPLES = 500;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;
    delay(2); // ~500 Hz during calibration
  }

  GyroOffset offset;
  offset.x = gx_sum / (float)NUM_SAMPLES;
  offset.y = gy_sum / (float)NUM_SAMPLES;
  offset.z = gz_sum / (float)NUM_SAMPLES;

  Serial.println("Gyro calibration complete.");
  Serial.printf("Offsets (raw): X=%.1f, Y=%.1f, Z=%.1f\n", offset.x, offset.y, offset.z);
  return offset;
}

// --------------------------------------------------------------------------------------
// Setup
// --------------------------------------------------------------------------------------
GyroOffset gyro_offset;

void setup() {
  // Initialize the serial communication
  Serial.begin(115200);
  while (!Serial)
    delay(10); // Will pause until serial console opens
  Serial.println("Adafruit MPU6050 test!");

  // IMU Setup---------------------------------------------------------------
  // Wire.begin() initializes I2C. For this wiring we are setting up out SDA
  // pin to be GPIO1 and our SCL pin to be GPIO2. Defualt pins were not
  // operating correctly due to conflictions with SPI pins.
  I2CIMU.begin(I2C_SDA, I2C_SCL, 400000);
  if (!mpu.begin(0x68, &I2CIMU)) {
    Serial.println("ERROR: MPU6050 connection failed! Check wiring.");
    // Halt here - there's no point continuing without the IMU
    while (true) {delay(1000);}
  }
  Serial.println("MPU6050 connected");

  // Set the MPU6050 Digital Low Pass Filter (DLPF) to reduce high-frequency
  // noise. A value of 3 gives a 44Hz bandwidth, which is appropriate for 
  // hand movement. Values: 0=260Hz, 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Set gyro full-scale range to +/- 250 deg/s (0 = +/-250, 1 = +/-500, 2 = +/-1000, 3 = +/-2000)
  // A lower range = more resolution, but clips if you move too fast.
  // +/-250 is fine for most write movements; increase to 1 if you see clipping.
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Set accelerometer full-scale range to +/- 2G
  // (0 = +/-2G, 1 = +/-4G, 2 = +/-8G, 3 = +/-16G)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // Calibrate the gyroscope offset before connecting to WiFi.
  gyro_offset = CalibrateGyro();

  // Madgwick Filter Setup-------------------------------------------------------
  // The sample rate must match how often you call filter.updateIMU() in loop().
  // The beta parameter controls how aggressively the filter corrects gyro
  // drift using the accelerometer. 0.1 is a typical starting value.
  // Higher beta = faster drift correction but more noise.
  filter.begin(SAMPLE_RATE_HZ);

  // WiFi and UDP Setup---------------------------------------------------------
  connectWiFi();

  last_sample_time_us = micros();
  Serial.println("Setup complete. Streaming packets...");
}
 
void loop() {
  // Enforce a fixed sample rate using microsecond timing.
  // This is more accurate than delay() because it accounts for the time
  // the rest of the loop body takes to execute.
  unsigned long now = micros();
  if (now - last_sample_time_us < SAMPLE_INTERVAL_US) {
    return; // Not time yet - yield back immediately
  }
  last_sample_time_us = now;

  // 1. Read raw IMU Data-------------------------------------------------------
  mpu.getEvent(&a, &g, &temp);
  
  // 2. Convert to physcial units, apply gyro calibration offset----------------
  // Gyro: subtract the at-rest offset before converting to deg/s
  // At +/-250 deg/s full scale, LSB sensitivity is 131.0 LSB/(deg/s)
  float gxf = (g.gyro.x - gyro_offset.x) / (180.0f / PI);
  float gyf = (g.gyro.y - gyro_offset.y) / (180.0f / PI);
  float gzf = (g.gyro.z - gyro_offset.z) / (180.0f / PI);

  // Accel: at +/-2G full scale, LSB sensitivity is 16384 LSB/g
  float axf = a.acceleration.x / 9.81f;
  float ayf = a.acceleration.y / 9.81f;
  float azf = a.acceleration.z / 9.81f;

  // 3. Update Madgwick Filter--------------------------------------------------
  // updateIMU() uses ony gyro + accel (no magnetometer).
  // Arguments order" gx, gy, gz (deg/s), ax, ay, az (any consistent unit)
  filter.updateIMU(gxf, gyf, gzf, axf, ayf, azf);

  // 4. Extract quaternion------------------------------------------------------
  float qw = filter.getQ0();
  float qx = filter.getQ1();
  float qy = filter.getQ2();
  float qz = filter.getQ3();

  // 5. Remove Gravity from Acceleration----------------------------------------
  // The raw accelerometer always measures gravity + linear motion.
  // To get just linear acceleration (needed for velocity integration later),
  // we subtract the gravity vector rotated into the sensor's local frame.
  // This is computed from the current quaternion.
  // Result is in g; multiply by 9.81 to convert to m/s^2.
  float ax_lin = (axf - 2.f * (qx*qz - qw*qy)) * 9.81f;
  float ay_lin = (ayf - 2.f * (qw*qx + qy*qz)) * 9.81f;
  float az_lin = (azf - (2.f * (0.5f - qx*qx - qy*qy))) * 9.81f;

  // 6. Build and send the Packet-----------------------------------------------
  ControllerPacket pkt;
  pkt.header = 0xAA;
  pkt.controller_id = CONTROLLER_ID;
  pkt.qw = qw; pkt.qx = qx; pkt.qy = qy; pkt.qz = qz;
  pkt.ax = ax_lin; pkt.ay = ay_lin; pkt.az = az_lin;

  // Position is filled in by the base stations on the PC size.
  // For now we send zeros; The driver will substitute base station data.
  pkt.x = 0.f; pkt.y = 0.f; pkt.z = 0.f;

  pkt.checksum = ComputeChecksum(pkt);

  udp.beginPacket(host_id, CONTROLLER_PORT);
  udp.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(ControllerPacket));
  udp.endPacket();

  // 7. Debug output over Serial (optional, comment out for performance)
  Serial.printf("q: %.3f %.3f %.3f %.3f | a: %.2f %.2f %.2f\n", 
    qw, qx, qy, qz, ax_lin, ay_lin, az_lin);
}
 
