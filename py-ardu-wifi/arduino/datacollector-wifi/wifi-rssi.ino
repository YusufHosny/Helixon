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
    byte *buf = WiFi.BSSID(i, BSSIDs[i]);
    for(int j = 0; j < 6; j++) {
      BSSIDs[i][j] = buf[j];
    }
  }
  printRssiData();
}

void printRssiData() {
  Serial.println(rssiCnt);
  for(int i = 0; i < (rssiCnt > 25 ? 25 : rssiCnt); i++) {
    Serial.print("(Reciever)"); Serial.print("SSID ("); Serial.print(i); Serial.print("): "); 
    for(int j = 0; j < 5; j++) {
      Serial.print(BSSIDs[i][j]); Serial.print(":");
    }
    Serial.print(BSSIDs[i][5]);
    
    Serial.print("RSSI: "); Serial.print(RSSIs[i]);
    Serial.println("dB");
  }
}