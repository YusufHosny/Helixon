#include "wifi-core.h"
#include "interboard-comms.h"

void setup_interboard(){
  Serial1.begin(115200);
}

// SENDER or MASTER has to be defined in header
#ifdef MASTER

int byteCnt = 0;
byte iboardbuf[601] = {}; // temporary data buffer
// THIS FUNCTION IS BLOCKING
void readInterboardRssis() {
  Serial1.write('r'); // request data

  // recv full data packet
  while (byteCnt < 601) {
    // block for data
    while(!Serial1.available());

    // read data into temp buffer
    iboardbuf[byteCnt++] = Serial1.read();
  }

  // process data
  int ssidIndex = 0;
  int charIndex = 0;
  byte rssiByte[4];
  int rssiByteCount = 0;

  for(int i = 0; i < 601; i++) {
    byte data = iboardbuf[i];

    if(i == 0) {
      rssiCnt = data;
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
          byteCnt = 0;
          ssidIndex = 0;
        }
      }
    }
  }
}

#endif // MASTER
#ifdef SLAVE

void sendInterboardRssis() {
  Serial1.write((byte *)&rssiCnt, 1);
  // Send SSID data
  for (int i = 0; i < 25; i++) {
    Serial1.write(SSIDs[i], 20);
    // Send RSSI data
    Serial1.write((byte *)&(RSSIs[i]), 4);
  }
  
  printRssiData();
}
#endif // SLAVE