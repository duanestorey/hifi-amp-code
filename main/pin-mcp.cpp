#include "pin-mcp.h"
#include "pin-mcp-manager.h"

PinMcp::PinMcp( PinMcpManager *pinManager, uint8_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt ) : mPin( pin ), mPinManager( pinManager ) {
    config( direction, pulldown, pullup, interrupt );
}


void 
PinMcp::config( uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt ) {
    if ( mPinManager ) {
    
    }
}

void 
PinMcp::setState( uint8_t state ) {
    if ( mPinManager ) {
        mPinManager->setState( mPin, state );
    }
}

uint8_t 
PinMcp::getState() const {
    uint8_t state = Pin::PIN_STATE_LOW;

    if ( mPinManager ) {
        state = mPinManager->getState( mPin );
    }

    return state;
}