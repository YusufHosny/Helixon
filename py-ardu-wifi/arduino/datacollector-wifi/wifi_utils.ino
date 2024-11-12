char ssid[] = "wifiFromNapoli";
char pass[] = "forzanapo";
// char ssid[] = "wifiFromLibanon";
// char pass[] = "libanonnumba1";
// char ssid[] = "RACIALLY MOTIVATED WIFI";
// char pass[] = "amogus12";

WiFiServer server(3435);
WiFiClient client = server.available();
int status = WL_IDLE_STATUS; // wifi status

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

void scanRSSIs() {
  // scan for all networks
  Serial.print("Scanning for all nearby wifi network RSSIs... ");
  int numFound = WiFi.scanNetworks();
  if(numFound == -1) {
    Serial.println("failed.");
    while(true);
  }
  Serial.println("success.");

  for(int i = 0; i < numFound; i++) {
    Serial.print(WiFi.SSID(i)); Serial.print(": "); Serial.print(WiFi.RSSI(i)); Serial.println("dBm");
  }
  Serial.println("----------------------------------------");
}

int8_t rssiCnt;
char SSIDs[25][20];
int32_t RSSIs[25]; 
void fillRssiData() {
  rssiCnt = WiFi.scanNetworks();
  for(int i = 0; i < 25 && i < rssiCnt; i++) {
    RSSIs[i] = WiFi.RSSI(i);
    char ssidBuf[20] = {}; 
    strncpy(ssidBuf, WiFi.SSID(i), 20);
    for(int j = 0; j < 20; j++) SSIDs[i][j] = ssidBuf[j];
  }
}

void startTCPServer() {
  server = WiFiServer(3435);
  server.begin();
}


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

void streamSwipes() {
  client = server.available();
  if (client) {                           
    Serial.println("new client");
    while (client.connected()) {  

      // read client data        
      while (client.available()) {
        char c = client.read();           
        Serial.write(c);                  
      }

      // get swipe state
      char s = getSwipe();

      // send swipe state
      client.print(s);
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void streamQuaternions() {
  client = server.available();
  if (client) {                           
    Serial.println("new client");
    while (client.connected()) {  

      // read client data        
      while (client.available()) {
        char c = client.read();           
        Serial.write(c);                  
      }

      // get quaternion
      imu::Quaternion q = getQuaternion();

      // send quaternion
      client.print("strt");
      client.write((byte *) &(q.w()), sizeof(q.w()));
      client.write((byte *) &(q.x()), sizeof(q.x()));
      client.write((byte *) &(q.y()), sizeof(q.y()));
      client.write((byte *) &(q.z()), sizeof(q.z()));
    }
    // close the connection:a
    client.stop();
    Serial.println("client disconnected");
  }
}

void streamDataEntries() {
  client = server.available();
  if (client) {                
    Serial.println("new client");
    digitalWrite(6, HIGH);    
    delay(1);
    while(client.connected()) {

      // read client request
      while (client.available()) {
        char c = client.read();           
        Serial.write(c);  
      }

      // get data from sensors
      DataEntry d = {};
      fillData(&d);

      // send data
      client.write((byte *) &d, sizeof(d));
      delay(4);
    }
    // close the connection
    client.stop();
    Serial.println("client disconnected");
    digitalWrite(6, LOW);    
  }
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


