#include "wifi-core.h"

void TCPrttTest() {
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


void UDPrttTest() {
  char packetBuffer[256]; // Buffer for incoming packets

  Serial.println("RTT test started");

  unsigned long start = micros();

  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write("t");
  udp.endPacket();

  while (!udp.parsePacket());  // Wait for a response

  udp.read(packetBuffer, sizeof(packetBuffer));
  unsigned long end = micros();

  if (packetBuffer[0] != 't') {
    Serial.print("Char mismatch received: ");
    Serial.println(packetBuffer[0]);
  }

  Serial.print("RTT test completed, RTT: ");
  Serial.print(end - start);
  Serial.println(" usec.");
}
