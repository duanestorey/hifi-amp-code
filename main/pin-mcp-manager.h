#ifndef __PIN_MCP_MANAGER_H__
#define __PIN_MCP_MANAGER_H__

#include <memory>
#include <map>
#include "pin-mcp.h"
#include "i2c-bus.h"

typedef std::map<uint8_t, PinPtr> PinMap;

class PinMcpManager {
    public:
        PinMcpManager( I2CBUSPtr bus, uint8_t addr, QueuePtr queue );
        
        uint8_t getState( uint8_t pin );
        void setState( uint8_t pin, uint8_t state );
        PinPtr createPin( uint8_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup );
        void updateConfig();

        void processPortAInterrupt();
        void processPortBInterrupt();
    protected:
        I2CBUSPtr mBus;
        PinMap mPinMap;
        uint8_t mAddr;

        bool isPortA( uint8_t pin );
        QueuePtr mInterruptQueue;
       
};

typedef std::shared_ptr<PinMcpManager> PinMcpManagerPtr; 

#endif