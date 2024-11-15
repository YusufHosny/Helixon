#ifndef INTERBOARDH
#define INTERBOARDH

#define RECEIVER
//#define SENDER

#include "wifi-core.h"
#include "interboard-comms.h"

void setup_i2c();

// SENDER or RECEIVER has to be defined
#ifdef RECEIVERSPI

#endif // RECEIVER
#ifdef SENDER

void sendI2CRssis();

#endif // SENDER

#endif