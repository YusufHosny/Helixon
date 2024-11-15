#include <Wire.h>

#include "wifi-core.h"
#include "sensors.h"
#include "types.h"
#include "spi-comms.h"

#ifdef RECEIVERSPI

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
  startCommandCenter();
}

#endif
#ifdef SENDERSPI

void setup() {
  delay(2000);
  Serial.begin(9600);

  enable_WiFi();

  setup_spi();
}

void loop() {   
  sendSpiRssis();
}

#endif
