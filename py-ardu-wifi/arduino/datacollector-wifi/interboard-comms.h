#ifndef INTERBOARDH
#define INTERBOARDH

#define RECEIVER
// #define SENDER

void setup_i2c();

// SENDER or RECEIVER has to be defined
#ifdef RECEIVER

#endif // RECEIVER
#ifdef SENDER

void sendI2CRssis();

#endif // SENDER

#endif