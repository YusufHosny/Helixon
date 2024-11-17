#include "wifi-core.h"
#include "interboard-comms.h"
#include <Wire.h>

// SENDER or RECEIVER has to be defined in header
#ifdef RECEIVER

void setup_i2c(){
  Wire.begin(70); 
  Wire.onReceive(receiveEvent);
}

int ssidIndex = 0;
int charIndex = 0;
bool first = true;
byte rssiByte[4];
int rssiByteCount = 0;
int byteCnt = 0;
void receiveEvent(int howmany){

  while (Wire.available()){
    byte data = Wire.read();
    Serial.write(data);
    byteCnt++;

    if(first) {
      rssiCnt = data;
      Serial.println(rssiCnt);
      first = false;
    }
    else if (charIndex < 20){
        SSIDs[ssidIndex][charIndex++] = data;
    }
    else {
      rssiByte[rssiByteCount++] = data;

      if(rssiByteCount == 4) {
        RSSIs[ssidIndex++] = *((int32_t *)rssiByte);
        charIndex = 0;
        rssiByteCount = 0;

        if(ssidIndex == 25) {
          Serial.print("total bytes: "); Serial.println(byteCnt);
          for(int i = 0; i < (rssiCnt > 25 ? 25 : rssiCnt); i++) {
            Serial.print("(Reciever)"); Serial.print("SSID ("); Serial.print(i); Serial.print("): "); Serial.print(SSIDs[i]);
            Serial.print("RSSI: "); Serial.print(RSSIs[i]);
            Serial.println("dB");
          }
          ssidIndex = 0;
          first = true;
        }
      }
    }
  }
}

#endif // RECEIVER
#ifdef SENDER

void setup_i2c(){
  Wire.begin(); // Join I2C bus with address
}

void sendI2CRssis() {
  // Populate SSIDs and RSSIs using fillRssiData
  fillRssiData();

  Wire.beginTransmission(70);
  
  Wire.write((byte *)&rssiCnt,1);
  // Send SSID data
  for (int i = 0; i < 25; i++) {
    Wire.write(SSIDs[i], 20);
    // Send RSSI data
    Wire.write((byte *)&(RSSIs[i]), 4);
  }

  Wire.endTransmission();
  Serial.println(rssiCnt);
  for(int i = 0; i < rssiCnt; i++) {
    Serial.print("SSID: "); Serial.print(SSIDs[i]);
    Serial.print("RSSI: "); Serial.print(RSSIs[i]);
    Serial.println("dB");
  }
}
#endif // SENDER