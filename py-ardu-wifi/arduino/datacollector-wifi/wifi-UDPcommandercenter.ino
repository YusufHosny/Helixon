#include "wifi-core.h"
#include "types.h"

bool streaming = false;  // Track streaming status
int packetCounter = 0;   // Counter to alternate between Wi-Fi and raw data

void startUDPCommandCenter() {
  char packetBuffer[257];  // Unified packet size of 257 bytes

  // Check for incoming commands
  int packetSize = udp.parsePacket();
  if (packetSize) {
    udp.read(packetBuffer, sizeof(packetBuffer));  // Read the incoming packet
    if (strncmp(packetBuffer, "strt", 4) == 0) {
      Serial.println("Start command received");
      streaming = true;
      digitalWrite(6, HIGH);  // Turn LED ON
    } else if (strncmp(packetBuffer, "stop", 4) == 0) {
      Serial.println("Stop command received");
      streaming = false;
      digitalWrite(6, LOW);  // Turn LED OFF
    }
  }

  // Stream data if streaming is active
  if (streaming) {
    if (packetCounter == 0) {  // Send Wi-Fi data
      packetBuffer[0] = 'w';  // Indicate Wi-Fi data
      readInterboardRssis();
      RssiDataEntry wifid;
      wifid.microsT = micros();
      wifid.rssiCnt = rssiCnt;
      for(int i = 0; i < 25; i++) {
          for(int j = 0; j < 6; j++) {
            wifid.BSSIDs[i][j] = BSSIDs[i][j];
          }
          wifid.RSSIs[i] = RSSIs[i];
        }
      memcpy(&packetBuffer[1], &wifid, sizeof(RssiDataEntry));
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write(packetBuffer, 257);
      udp.endPacket();
    } else {  // Send raw data
      packetBuffer[0] = 'r';  // Indicate raw data
      DataEntry d1 = {}, d2 = {};
      fillData(&d1);
      fillData(&d2);  // Generate two packets of sensor data
      memcpy(&packetBuffer[1], &d1, 128);
      memcpy(&packetBuffer[129], &d2, 128);
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write(packetBuffer, 257);
      udp.endPacket();
    }

    // Alternate between Wi-Fi and raw data
    packetCounter = (packetCounter + 1) % 100;

    delay(20);  // Adjust delay for streaming rate
  }
}