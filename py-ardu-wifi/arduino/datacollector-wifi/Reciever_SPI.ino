#include <SPI.h>

char SSIDs[25][20];
int32_t RSSIs[25];

volatile byte receivedData;
volatile int ssidIndex = 0;
volatile int charIndex = 0;
volatile int rssiIndex = 0;
volatile byte rssiByte[4];
volatile int rssiByteCount = 0;

void setup() {
    SPI.begin(); // Start SPI as Slave
    SPI.attachInterrupt(); // Enable interrupt
    Serial.begin(9600);
}

ISR(SPI_STC_vect) { // SPI Interrupt Service Routine
    receivedData = SPDR; // Read the received byte

    if (ssidIndex < 25) {
        SSIDs[ssidIndex][charIndex++] = receivedData;
        if (charIndex == 20) { // Reached end of one SSID
            charIndex = 0;
            ssidIndex++;
        }
    } else if (rssiIndex < 25) {
        rssiByte[rssiByteCount++] = receivedData;
        if (rssiByteCount == 4) { // Collected 4 bytes for one int32_t RSSI
            RSSIs[rssiIndex] = *((int32_t *)rssiByte);
            rssiByteCount = 0;
            rssiIndex++;
        }
    }
}

void loop() {
    if (ssidIndex == 25 && rssiIndex == 25) {
        // Print data to verify successful reception
        for (int i = 0; i < 25; i++) {
            Serial.print("SSID ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(SSIDs[i]);
            Serial.print("RSSI ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(RSSIs[i]);
        }
        
        ssidIndex = 0; // Reset indices for next data set
        rssiIndex = 0;
        delay(1000); // Wait before next read
    }
}
