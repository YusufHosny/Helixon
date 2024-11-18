#include "wifi-core.h"

void scanRSSIs() {
  // scan for all networks
  Serial.print("Scanning for all nearby wifi network RSSIs... ");
  fillRssiData();
  printRssiData();
  Serial.println("----------------------------------------");
}

void fillRssiData() {
  rssiCnt = WiFi.scanNetworks();
  if(rssiCnt == -1) {
    Serial.println("rssi scan failed.");
  }
  for(int i = 0; i < 25 && i < rssiCnt; i++) {
    RSSIs[i] = WiFi.RSSI(i);
    char ssidBuf[20] = {}; 
    strncpy(ssidBuf, WiFi.SSID(i), sizeof(ssidBuf));
    memcpy(SSIDs[i], ssidBuf, sizeof(SSIDs[i]));
  }
}

void printRssiData() {
  Serial.println(rssiCnt);
  for(int i = 0; i < (rssiCnt > 25 ? 25 : rssiCnt); i++) {
    Serial.print("(Reciever)"); Serial.print("SSID ("); Serial.print(i); Serial.print("): "); Serial.print(SSIDs[i]);
    Serial.print("RSSI: "); Serial.print(RSSIs[i]);
    Serial.println("dB");
  }
}