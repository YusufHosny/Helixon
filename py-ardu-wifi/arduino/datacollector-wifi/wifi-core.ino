#include "wifi-core.h"

void enable_WiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }
}

void connect_WiFi() {
  // attempt connection
  do {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  } while (status != WL_CONNECTED);

  Serial.println("Connection Successful.");
  Serial.println("----------------------------------------");
}

void disconnect_WiFi() {
  WiFi.disconnect();
  WiFi.end();
  Serial.println("Disconnect Successful.");
  Serial.println("----------------------------------------");
}


void startTCPServer() {
  server = WiFiServer(3435);
  server.begin();
}
void startUDPServer() {  // New function for UDP
  udp.begin(3435);
  Serial.println("UDP server started on port 3435.");
}

void printWifiData() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("rssi: ");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println("----------------------------------------");
}
