#include <Wire.h>
#include "interboard-comms.h"
#include "wifi-core.h"
#include "sensors.h"
#include "types.h"

#ifdef MASTER

void setup() {
  delay(2000);
  Serial.begin(9600);

  pinMode(6, OUTPUT);  // Set the LED pin as an output

  enable_WiFi();
  connect_WiFi();

  // startTCPServer();
  startUDPServer();

  printWifiData();

  setup_interboard();

  setup_bno();
  setup_bmp();
}

void loop() {
  startUDPCommandCenter();
  // startTCPCommandCenter();
}

#endif
#ifdef SLAVE
long unsigned int tprev;

void setup() {
  delay(2000);
  Serial.begin(9600);

  enable_WiFi();

  setup_interboard();
  
  tprev = millis();
}

void loop() {
  if(millis() - tprev > 3000) { // every 3 sec
    fillRssiData();
    tprev = millis();
  }
    
  if(Serial1.available() && Serial1.read() == 'r')
    sendInterboardRssis();
    fillRssiData();
    tprev = millis();
}

#endif
