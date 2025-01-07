#ifndef __PIN_MANAGER_H__
#define __PIN_MANAGER_H__

#include "abstract/pin.h"
#include "config.h"
#include "queue.h"
#include "pin-mcp-manager.h"
#include "i2c-bus.h"
#include <map>
#include <memory>

struct ESP32_IntData;

typedef std::map<uint32_t,ESP32_IntData *> InterruptMap;

class PinManager {
    public:
        enum {
            PIN_TYPE_ESP32 = 0,
            PIN_TYPE_MCP = 1
        };

        PinManager( I2CBUSPtr i2c, QueuePtr interruptQueue );
        PinPtr createPin( uint8_t pinType, uint8_t pinReference, uint8_t direction, uint8_t pulldown, uint8_t pullup );
        void enableInterrupt( uint8_t pinReference, uint8_t interruptType  );

        void handleInterrupt( uint8_t pin );
        void handleMcpPortA();
        void handleMcpPortB();

        void _handlePortA();
        void _handlePortB();
    protected:
        QueuePtr mInterruptQueue;
        PinMcpManagerPtr mPinManagerMCP;
        I2CBUSPtr mI2C;
        InterruptMap mInterrupts;

    private:
        void configureMCPInterrupts();
};

struct ESP32_IntData {
    ESP32_IntData() : mPin( 0 ), mPinManager( 0 ) {}
    ESP32_IntData( uint8_t pin, PinManager *pinManager ) : mPin( pin ), mPinManager( pinManager ) {}

    uint8_t mPin;
    PinManager *mPinManager;
};


typedef std::shared_ptr<PinManager> PinManagerPtr;

#endif