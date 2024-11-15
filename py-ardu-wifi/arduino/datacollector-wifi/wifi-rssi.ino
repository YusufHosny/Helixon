#include "wifi-core.h"

void scanRSSIs() {
  // scan for all networks
  Serial.print("Scanning for all nearby wifi network RSSIs... ");
  int numFound = WiFi.scanNetworks();
  if(numFound == -1) {
    Serial.println("failed.");
    while(true);
  }
  Serial.println("success.");

  for(int i = 0; i < numFound; i++) {
    Serial.print(WiFi.SSID(i)); Serial.print(": "); Serial.print(WiFi.RSSI(i)); Serial.println("dBm");
  }
  Serial.println("----------------------------------------");
}


void fillRssiData() {
  delay(5000);
  rssiCnt = WiFi.scanNetworks();
  for(int i = 0; i < 25 && i < rssiCnt; i++) {
    RSSIs[i] = WiFi.RSSI(i);
    char ssidBuf[20] = {}; 
    strncpy(ssidBuf, WiFi.SSID(i), 20);
    for(int j = 0; j < 20; j++) SSIDs[i][j] = ssidBuf[j];
  }
}