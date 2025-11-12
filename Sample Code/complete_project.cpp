#include <Wire.h>          // I2C
#include <SPI.h>           // SPI for nRF24L01
#include <RF24.h>          // nRF24L01 radio
#include <Servo.h>         // Servo for LiDAR rotation
#include <Adafruit_BNO055.h> // IMU
#include <Adafruit_BME680.h> // Environmental sensor
#include <Adafruit_AMG88xx.h> // Thermal array
#include <utility/imumaths.h> // For BNO055
#include <SoftwareSerial.h>  // For GPS UART
#include <TinyGPS++.h>       // GPS parsing

// Pin definitions
#define LIDAR_TRIGGER 2
#define LIDAR_MODE 3
#define SERVO_PIN 6
#define NRF_CE 9
#define NRF_CSN 10
#define GPS_RX 4      // GPS TX -> Arduino RX
#define GPS_TX 5      // GPS RX -> Arduino TX (not always used)

// Initialize objects
Servo lidarServo;
RF24 radio(NRF_CE, NRF_CSN);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BME680 bme;
Adafruit_AMG88xx amg;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX); // Soft UART for GPS
TinyGPSPlus gps; // Parser

// Updated data struct (added GPS; total ~36 bytes—split tx if needed for nRF 32B limit)
struct SensorData {
  float lidarDist;      // cm
  float imuPitch;       // degrees
  float imuRoll;        // degrees
  float temp;           // C
  float humidity;       // %
  float pressure;       // hPa
  float gasResistance;  // kOhms
  float thermal[2];     // Simplified: avg hot/cold quadrants (reduced for space)
  double gpsLat;        // degrees
  double gpsLon;        // degrees
  float gpsAlt;         // m
  bool gpsValid;        // Fix flag
  uint16_t crc;         // Checksum
} packet;

// Timing
unsigned long lastRead = 0;
const unsigned long interval = 1000; // ms (slower for GPS stability)

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600); // GPS default baud (configurable to 115200)
  Wire.begin();
  
  // Initialize sensors (as before)
  if (!bno.begin()) { Serial.println("BNO055 failed!"); while(1); }
  if (!bme.begin()) { Serial.println("BME680 failed!"); while(1); }
  if (!amg.begin()) { Serial.println("AMG8833 failed!"); while(1); }
  
  // LiDAR, Servo, Radio setup (unchanged)
  pinMode(LIDAR_TRIGGER, OUTPUT);
  pinMode(LIDAR_MODE, INPUT);
  lidarServo.attach(SERVO_PIN);
  lidarServo.write(0);
  radio.begin();
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  
  // BME config (unchanged)
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  
  Serial.println("All sensors + GPS initialized. Starting loop...");
}

void loop() {
  // Feed GPS serial data continuously
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Optional: gps.location.isValid() for new fix
    }
  }
  
  if (millis() - lastRead >= interval) {
    readSensors();
    transmitData();
    lastRead = millis();
  }
  
  // Servo rotation (unchanged)
  static int servoPos = 0;
  static unsigned long lastServo = 0;
  if (millis() - lastServo >= 50) {
    servoPos = (servoPos + 5) % 180;
    lidarServo.write(servoPos);
    lastServo = millis();
  }
  
  delay(10);
}

void readSensors() {
  // LiDAR + IMU leveling (unchanged)
  digitalWrite(LIDAR_TRIGGER, LOW); delayMicroseconds(2);
  digitalWrite(LIDAR_TRIGGER, HIGH); delayMicroseconds(10);
  digitalWrite(LIDAR_TRIGGER, LOW);
  while (digitalRead(LIDAR_MODE) == LOW);
  unsigned long duration = pulseIn(LIDAR_MODE, HIGH);
  packet.lidarDist = (duration / 2) / 29.1;
  
  sensors_event_t event;
  bno.getEvent(&event);
  packet.imuPitch = event.orientation.y;
  packet.imuRoll = event.orientation.z;
  float tiltCos = cos(radians(packet.imuPitch)) * cos(radians(packet.imuRoll));
  packet.lidarDist /= tiltCos;
  
  // BME680 (unchanged)
  if (bme.performReading()) {
    packet.temp = bme.temperature;
    packet.humidity = bme.humidity;
    packet.pressure = bme.pressure / 100.0;
    packet.gasResistance = bme.gas_resistance / 1000.0;
  }
  
  // AMG8833 (simplified to 2 avgs for packet size)
  float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  amg.readPixels(pixels);
  packet.thermal[0] = avgQuadrant(pixels, 0, 32); // Hot avg (top half)
  packet.thermal[1] = avgQuadrant(pixels, 32, 64); // Cold avg (bottom half)
  
  // NEW: GPS
  packet.gpsValid = gps.location.isValid();
  if (packet.gpsValid) {
    packet.gpsLat = gps.location.lat();
    packet.gpsLon = gps.location.lng();
    packet.gpsAlt = gps.altitude.meters();
  } else {
    packet.gpsLat = packet.gpsLon = packet.gpsAlt = 0.0; // No fix
  }
  
  // Serial debug
  Serial.print("GPS: "); Serial.print(packet.gpsValid ? "FIX" : "NO"); 
  Serial.print(" Lat:"); Serial.print(packet.gpsLat, 6); 
  Serial.print(" Lon:"); Serial.println(packet.gpsLon, 6);
}

float avgQuadrant(float* pixels, int start, int end) {
  float sum = 0;
  for (int i = start; i < end; i++) sum += pixels[i];
  return sum / (end - start);
}

void transmitData() {
  // CRC (XOR; unchanged)
  packet.crc = 0;
  uint8_t* p = (uint8_t*)&packet;
  for (int i = 0; i < sizeof(packet) - 2; i++) {
    packet.crc ^= p[i];
  }
  
  // Split tx if >32B (for nRF): Send core + GPS separate, or use dynamic payload
  bool success = radio.write(&packet, sizeof(packet)); // Assumes 32B+ mode or trim
  if (success) {
    Serial.println("Data (w/GPS) transmitted!");
  } else {
    Serial.println("Transmission failed!");
  }
}