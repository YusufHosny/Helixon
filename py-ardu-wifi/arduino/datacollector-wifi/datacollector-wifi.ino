#include <Wire.h>
#include "interboard-comms.h"
#include "wifi-core.h"
#include "sensors.h"
#include "types.h"

#ifdef RECEIVER

void setup() {
  delay(2000);
  Serial.begin(9600);

  pinMode(6, OUTPUT);  // Set the LED pin as an output

  enable_WiFi();
  connect_WiFi();
  startTCPServer();
  printWifiData();

  setup_i2c();

  // setup_bno();
  // setup_bmp();
}

void loop() {
  startCommandCenter();
}

#endif
#ifdef SENDER

void setup() {
  delay(2000);
  Serial.begin(9600);

  enable_WiFi();

  setup_i2c();
}

void loop() {   
  sendI2CRssis();
}

#endif
