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

char SSIDs[25][20];
int32_t RSSIs[25];

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

  pinMode(SS, OUTPUT);  // Set Slave Select as output
  digitalWrite(SS, HIGH);  // Initially keep SS high

  Serial.begin(9600);
  delay(1000);
}

void loop() {
  //streamDataEntries();

  // Populate SSIDs and RSSIs using fillRssiData
  fillRssiData(SSIDs, RSSIs);
  
  // Begin SPI communication with the Slave
  digitalWrite(SS, LOW);  // Select Slave

  // Send SSID data
  for (int i = 0; i < 25; i++) {
      for (int j = 0; j < 20; j++) {
          SPI.transfer(SSIDs[i][j]);
      }
  }

  // Send RSSI data
  for (int i = 0; i < 25; i++) {
      byte *bytePointer = (byte *)&RSSIs[i];
      for (int k = 0; k < 4; k++) {
          SPI.transfer(bytePointer[k]);  // Send each byte of the int32_t RSSI
      }
  }

  digitalWrite(SS, HIGH);  // Deselect Slave

  delay(1000);  // Delay between transmissions
}
