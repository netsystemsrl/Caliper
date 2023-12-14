/Users/utente/Library/Arduino15/packages/esp32/hardware/esp32/2.0.10/cores/esp32/Server.h


#ifndef server_h
#define server_h

#include "Print.h"

class Server: public Print
{
public:
    virtual void begin(uint16_t port=0) =0;
};

class ServerEth: public Print
{
public:
    //virtual void begin(uint16_t port=0) =0;
};
#endif
