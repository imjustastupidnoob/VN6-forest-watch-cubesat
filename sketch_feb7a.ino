#include <Adafruit_INA219.h>
#include <SPI.h>
#include <WebServer.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <LoRa.h>
#include <QMC5883LCompass.h>
#include <Adafruit_BusIO_Register.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MLX90640.h>
#include <math.h>

Adafruit_MLX90640 mlx;

float frame[32*24]; // buffer for one frame


// Sensor objects
Adafruit_INA219 ina219;
Adafruit_MPU6050 mpu;
Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();
QMC5883LCompass compass;
TinyGPSPlus gps;
Adafruit_BMP280 bmp;
HardwareSerial Sgps(1);
// Networking
WebServer server(80);

// Example: WiFi credentials
const char* ssid = "VN6 cubesat";
const char* password = "VN6 iS tHe beSt";

// LoRa pins (adjust for your board)
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26
const double EARTH_RADIUS = 6371000.0;
const double h = 20000.0;
// Convert lat/lon/alt to ECEF
void latLonAltToECEF(double lat, double lon, double alt, double &x, double &y, double &z) {
  lat = radians(lat);
  lon = radians(lon);
  double r = EARTH_RADIUS + alt;
  x = r * cos(lat) * cos(lon);
  y = r * cos(lat) * sin(lon);
  z = r * sin(lat);
}

// Find intersection of line of sight with Earth surface
bool lineOfSightIntersection(double sx, double sy, double sz,
                             double dx, double dy, double dz,
                             double &ix, double &iy, double &iz) {
  // Quadratic solution: |S + tD|^2 = R^2
  double a = dx*dx + dy*dy + dz*dz;
  double b = 2*(sx*dx + sy*dy + sz*dz);
  double c = sx*sx + sy*sy + sz*sz - EARTH_RADIUS*EARTH_RADIUS;

  double disc = b*b - 4*a*c;
  if (disc < 0) return false; // no intersection

  double t = (-b - sqrt(disc)) / (2*a); // nearest intersection
  if (t < 0) return false; // pointing away

  ix = sx + t*dx;
  iy = sy + t*dy;
  iz = sz + t*dz;
  return true;
}

// Convert ECEF back to lat/lon
void ecefToLatLon(double x, double y, double z, double &lat, double &lon) {
  lat = degrees(atan2(z, sqrt(x*x + y*y)));
  lon = degrees(atan2(y, x));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Sgps.begin(9600, SERIAL_8N1, 16, 17);
  // Power monitor
  if (!ina219.begin()) {
    Serial.println("INA219 not found");
  }

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
  }

  // Compass
  compass.init();

  // BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found");
  }

  // VL53L0X distance sensor
  if (!vl53.begin()) {
    Serial.println("VL53L0X not found");
  }

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Web server route
  server.on("/", []() {
    server.send(200, "text/plain", "Satellite ESP32-S2 running");
  });
  server.begin();

  // LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed");
  }
  Wire.begin();
  if (!mlx.begin()) {
    Serial.println("MLX90640 not found!");
    while (1);
  }

}

void loop() {
  server.handleClient();

  // Example: read sensors
  float shuntvoltage = ina219.getShuntVoltage_mV();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  compass.read();
  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();

  float pressure = bmp.readPressure();
  VL53L0X_RangingMeasurementData_t measure;
  vl53.rangingTest(&measure, false);

  // Example: send data via LoRa
  LoRa.beginPacket();
  if(temp.temperature > 150){
    LoRa.print("warning!!!!! \n temperature is hot in some area!!!!!, this area is potential to catch fire: ");
  }
  if (mlx.getFrame(frame)) {
    for (int y = 0; y < 24; y++) {
      for (int x = 0; x < 32; x++) {
        float temp = frame[y * 32 + x];
        LoRa.print(temp);
        int cnt = 0;
        if(y != 0) cnt += (frame[(y + 1) * 32 + x] > 150);
        if(y != 23) cnt += (frame[(y - 1) * 32 + x] > 150);
        if(x != 0) cnt += (frame[y * 32 + (x - 1)] > 150);
        if(x != 31) cnt += (frame[y * 32 + (x  + 1)] > 150);
        if(x != 0 and y != 0) cnt += (frame[(y - 1) * 32 + (x - 1)]  > 150);
        if(x != 0 and y != 23) cnt += (frame[(y + 1) * 32 + (x - 1)]  > 150);
        if(x != 31 and y != 0) cnt += (frame[(y - 1) * 32  + x + 1]  > 150);
        if(x != 31 and y != 23) cnt += (frame[(y + 1) * 32 + x + 1]  > 150);
        LoRa.print(" ");
        if(cnt > 3){
          LoRa.print("lots of hot points in this area! \n");// earth radius: EARTH_RADIUS, height: h
          double sx = (EARTH_RADIUS + h) * cos(gps.location.lat()) * cos(gps.location.lng()), sy = (EARTH_RADIUS + h) * cos(gps.location.lat()) * sin(gps.location.lng()), sz = (EARTH_RADIUS + h) * sin(gps.location.lat());
          latLonAltToECEF(gps.location.lat(), gps.location.lng(), h, sx, sy, sz);

          // Example pointing vector (downward)
          double dx = -sx; 
          double dy = -sy;
          double dz = -sz;

          double ix, iy, iz;
          if (lineOfSightIntersection(sx, sy, sz, dx, dy, dz, ix, iy, iz)) {
            double lat, lon;
            ecefToLatLon(ix, iy, iz, lat, lon);
            Serial.print("Pointing at Lat: "); Serial.println(lat, 6);
            Serial.print("Lon: "); Serial.println(lon, 6);
          }

        }

      }
      Serial.println();
    }
  }

  LoRa.print("Temp: ");
  LoRa.print(temp.temperature);
  LoRa.print(" Pressure: ");
  LoRa.print(pressure);
  LoRa.endPacket();

  delay(2000);
}