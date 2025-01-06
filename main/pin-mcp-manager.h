#ifndef __PIN_MCP_MANAGER_H__
#define __PIN_MCP_MANAGER_H__

#include <memory>
#include "pin-mcp.h"
#include "i2c-bus.h"

class PinMcpManager {
    public:
        PinMcpManager( I2CBUSPtr bus );
        void registerPin( PinMcp *pin );
        uint8_t getState( uint8_t pin );
        void setState( uint8_t pin, uint8_t state );
        PinPtr createPin( uint8_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt = Pin::PIN_INT_DISABLE );
       
    protected:
        PinMcp *mPins[ PinMcp::PIN_B7 + 1 ];
        I2CBUSPtr mBus;
};

typedef std::shared_ptr<PinMcpManager> PinMcpManagerPtr; 

#endif