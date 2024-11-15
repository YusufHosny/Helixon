#include "wifi-core.h"
#include "interboard-comms.h"

// SENDER or RECEIVER has to be defined in header
#ifdef RECEIVER

volatile byte receivedData;
volatile int ssidIndex = 0;
volatile int charIndex = 0;
volatile int rssiIndex = 0;
volatile byte rssiByte[4];
volatile int rssiByteCount = 0;

// THIS CODE READS INTO BUFFERS, USING THE DATA VARIABLE AS THE READ BYTE
// CALL THIS EVERY TIME A BYTE IS RECVD
if (ssidIndex < 25) {
    SSIDs[ssidIndex][charIndex++] = data;
    if (charIndex == 20) { // Reached end of one SSID
        charIndex = 0;
    }
} else if (rssiIndex < 25) {
    rssiByte[rssiByteCount++] = data;
    if (rssiByteCount == 4) { // Collected 4 bytes for one int32_t RSSI
        RSSIs[rssiIndex] = *((int32_t *)rssiByte);
        rssiByteCount = 0;
        for(int i = 0; i < rssiCnt; i++) {
          Serial.print("SSID: "); Serial.print(SSIDs[i]);
          Serial.print("RSSI: "); Serial.print(RSSIs[i]);
          Serial.println("dB");
        }
    }
}


#endif // RECEIVER
#ifdef SENDER

void sendI2CRssis() {
  // Populate SSIDs and RSSIs using fillRssiData
  fillRssiData();

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

  for(int i = 0; i < rssiCnt; i++) {
    Serial.print("SSID: "); Serial.print(SSIDs[i]);
    Serial.print("RSSI: "); Serial.print(RSSIs[i]);
    Serial.println("dB");
  }

}


#endif // SENDERSPI