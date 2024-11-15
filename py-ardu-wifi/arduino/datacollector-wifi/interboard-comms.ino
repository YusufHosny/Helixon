#include "wifi-core.h"
#include "interboard-comms.h"
#include <Wire.h>

// SENDER or RECEIVER has to be defined in header
#ifdef RECEIVER

void setup_i2c(){
  Wire.begin(70); 
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}

int ssidIndex = 0;
int charIndex = 0;
bool first = true;
byte rssiByte[4];
int rssiByteCount = 0;

void receiveEvent(int howmany){

  while (Wire.available()){
    // char c = Wire.read(); // Receive byte by byte
    // Serial.print(c); // Print received data to Serial Monitor
    // THIS CODE READS INTO BUFFERS, USING THE DATA VARIABLE AS THE READ BYTE
    // CALL THIS EVERY TIME A BYTE IS RECVD

    if(first) {
      rssiCnt = Wire.read();
      Serial.println(rssiCnt);
      first = false;
    }
    else {
      if (charIndex < 20){
        SSIDs[ssidIndex][charIndex++] = Wire.read();
      }
      else if(rssiByteCount < 4) {
        rssiByte[rssiByteCount++] = Wire.read();
      }
      else {
        RSSIs[ssidIndex++] = *((int32_t *)rssiByte);
        charIndex = 0;
        rssiByteCount = 0;
        if(ssidIndex == 25) {
          for(int i = 0; i < (rssiCnt > 25 ? 25 : rssiCnt); i++) {
            Serial.print("(Reciever) SSID: "); Serial.print(SSIDs[i]);
            Serial.print("(Reciever) RSSI: "); Serial.print(RSSIs[i]);
            Serial.println("dB");
          }
          ssidIndex = 0;
          charIndex = 0;
          first = true;
          rssiByteCount = 0;
        }
      }
    }
  }
}

#endif // RECEIVER
#ifdef SENDER

void setup_i2c(){
  Wire.begin(); // Join I2C bus with address 8
  Serial.begin(9600);
}

void sendI2CRssis() {
  // Populate SSIDs and RSSIs using fillRssiData
  fillRssiData();

  Wire.beginTransmission(70);
  Serial.print(rssiCnt);
  Wire.write((byte *)&rssiCnt,1);
  // Send SSID data
  for (int i = 0; i < 25; i++) {
    Wire.write(SSIDs[i],20);
    // Send RSSI data
    Wire.write((byte *)&(RSSIs[i]), 4);  // Send each byte of the int32_t RSSI
  }  

  Wire.endTransmission();

  for(int i = 0; i < rssiCnt; i++) {
    Serial.print("SSID: "); Serial.print(SSIDs[i]);
    Serial.print("RSSI: "); Serial.print(RSSIs[i]);
    Serial.println("dB");
  }
}
#endif // SENDER