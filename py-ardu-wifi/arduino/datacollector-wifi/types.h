#ifndef HLXTYPESH
#define HLXTYPESH

struct DataEntry { // size 128bytes
  unsigned long microsT; // 4 bytes + 4 padding
  double accelx, accely, accelz; // 8 bytes * 3
  double gyrox, gyroy, gyroz; // 8 bytes * 3
  double magnx, magny, magnz; // 8 bytes * 3
  double roll, pitch, yaw; // 8 bytes * 3
  int8_t tempbno;  // 1 byte + 7 padding
  double tempbmp; // 8 bytes
  double pressure; // 8 bytes
};

#endif