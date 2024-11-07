#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>

struct DataEntry { // size 128bytes
  unsigned long microsT; // 4 bytes + 4 padding
  double accelx, accely, accelz; // 8 bytes * 3
  double gyrox, gyroy, gyroz; // 8 bytes * 3
  double magnx, magny, magnz; // 8 bytes * 3
  double roll, pitch, yaw; // 8 bytes * 3
  int8_t tempbno;  // 1 byte + 7 padding
  double tempbmp; // 8 bytes
  double pressure; // 8 bytes
};

void setup() {
  delay(2000);
  Serial.begin(9600);

  pinMode(6, OUTPUT);  // Set the LED pin as an output

  enable_WiFi();
  connect_WiFi();
  startTCPServer();
  printWifiData();

  setup_bno();
  setup_bmp();
}

void loop() {
  streamDataEntries();
}
