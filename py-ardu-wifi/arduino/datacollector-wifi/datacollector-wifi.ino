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
  startTCPServer();
  printWifiData();

  setup_interboard();

  setup_bno();
  setup_bmp();
}

void loop() {
  readInterboardRssis();
}

#endif
#ifdef SLAVE

void setup() {
  delay(2000);
  Serial.begin(9600);

  enable_WiFi();

  setup_interboard();
}

void loop() {   
  // nothing
  fillRssiData();
  sendInterboardRssis();
}

#endif
