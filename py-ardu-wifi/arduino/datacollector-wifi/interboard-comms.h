#ifndef INTERBOARDH
#define INTERBOARDH

#define MASTER
//#define SLAVE

void setup_interboard();

// SLAVE or MASTER has to be defined
#ifdef MASTER

void readInterboardRssis();

#endif // MASTER
#ifdef SLAVE

void sendInterboardRssis();

#endif // SLAVE

#endif