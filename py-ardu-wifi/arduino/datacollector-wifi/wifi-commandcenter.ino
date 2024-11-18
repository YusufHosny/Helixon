#include "wifi-core.h"
#include "types.h"

void startCommandCenter() {
  client = server.available();
  if (client) {                
    Serial.println("new client");
    digitalWrite(6, HIGH);    

    char request[4] = {};
    while(client.connected()) {
      while (client.available()) {
        // read next char
        char c = client.read();
        Serial.write(c);
        
        // shift in new char
        for(unsigned char i = 1; i < 4; i++) {
          request[i-1] = request[i];
        }
        request[3] = c;
         
        // check which request
        if(strncmp(request, "data", 4) == 0) {
          // get data from sensors
          DataEntry d = {};
          fillData(&d);

          // send data
          client.write((byte *) &d, sizeof(d));
        }
        else if(strncmp(request, "wifi", 4) == 0) {
          // update rssi vals
          readInterboardRssis();

          RssiDataEntry wifid;
          wifid.rssiCnt = rssiCnt;
          for(int i = 0; i < 25; i++) {
            for(int j = 0; j < 20; j++) {
              wifid.SSIDs[i][j] = SSIDs[i][j];
            }
            wifid.RSSIs[i] = RSSIs[i];
          }

          client.write((byte *) &wifid, sizeof(wifid));
          Serial.print("done");
        }
      }
    }
    // close the connection
    client.stop();
    Serial.println("client disconnected");
    digitalWrite(6, LOW);    
  }
}




