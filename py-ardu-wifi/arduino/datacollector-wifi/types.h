#ifndef HLXTYPESH
#define HLXTYPESH

struct DataEntry { // size 152 bytes
  unsigned long microsT; // 4 bytes + 4 padding
  double linaccelx, linaccely, linaccelz; // 8 bytes * 3
  double gyrox, gyroy, gyroz; // 8 bytes * 3
  double magnx, magny, magnz; // 8 bytes * 3
  double roll, pitch, yaw; // 8 bytes * 3
  int8_t tempbno;  // 1 byte + 7 padding
  double tempbmp; // 8 bytes
  double pressure; // 8 bytes
};

struct RssiDataEntry { // size 252 bytes 
  int8_t rssiCnt; // 1 byte
  byte BSSIDs[25][6]; // 25*6 = 150 bytes + 2 padding
  int32_t RSSIs[25]; // 25*4 = 100 bytes
};

#endif