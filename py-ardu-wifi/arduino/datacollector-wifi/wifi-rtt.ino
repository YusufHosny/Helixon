#include "wifi-core.h"

void rttTest() {
  if (client) {                           
    Serial.println("rtt test started");

    // check for test start chars
    if (client.available()) {
      char recvd = client.read();
      client.read();

      if(recvd != 'r') {
        Serial.println("invalid rtttest char");
        return;
      }
    }
    // packet send and recv times (microsec)
    unsigned long start, end;
    delay(10);
    start = micros();
    // send packet
    client.print("t");

    // wait for response
    while (!client.available());
    char recvd = client.read();
    end = micros();
    if(recvd != 't') {
      Serial.print("char mismatch recvd: ");
      Serial.println(recvd);
    }

    client.stop();
    Serial.print("rtt test completed, rtt: ");
    Serial.print(end-start);
    Serial.println(" usec.");
  }
}
