#include "wifi-core.h"
#include "types.h"

void startCommandCenter() {
  client = server.available();
  if (client) {                
    Serial.println("new client");
    digitalWrite(6, HIGH);    
    delay(1);
    while(client.connected()) {

      // read client request
      char request[4] = {};
      while (client.available()) {
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
          delay(4);
        }
        else if(strncmp(request, "wifi", 4) == 0) {
          // 

        }
      }
    }
    // close the connection
    client.stop();
    Serial.println("client disconnected");
    digitalWrite(6, LOW);    
  }
}




