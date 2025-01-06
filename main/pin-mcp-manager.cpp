#include "pin-mcp-manager.h"
#include <memory.h>

PinMcpManager::PinMcpManager( I2CBUSPtr bus ) : mBus( bus ) {
    memset( mPins, 0, sizeof( PinMcp *) * ( PinMcp::PIN_B7 + 1 ) );
}

void 
PinMcpManager::registerPin( PinMcp *pin ) {
    mPins[ pin->getPinID() ] = pin;
}

uint8_t 
PinMcpManager::getState( uint8_t pin ) {
    uint8_t state = 0;

    return state;
}

void 
PinMcpManager::setState( uint8_t pin, uint8_t state ) {
    // update state here
}

PinPtr 
PinMcpManager::createPin( uint8_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt ) {
    PinPtr newPtr = PinPtr( new PinMcp( this, pin, direction, pulldown, pullup, interrupt ) );

    return newPtr;
}

