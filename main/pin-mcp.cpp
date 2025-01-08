#include "pin-mcp.h"
#include "pin-mcp-manager.h"
#include "debug.h"

PinMcp::PinMcp( PinMcpManager *pinManager, uint8_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup ) : Pin( direction, pulldown, pullup ), mPin( pin ), mPinManager( pinManager ) {
    config( direction, pulldown, pullup );
}

void 
PinMcp::enableInterrupt( uint8_t interruptType ) {
    if ( interruptType == Pin::PIN_INT_BOTH ) {
        AMP_DEBUG_E( "Unsupprted MCP interrupt type, PIN_INT_BOTH" );
    } else {
        if ( interruptType != Pin::PIN_INT_DISABLE ) {
            Pin::enableInterrupt( interruptType );
            mPinManager->updateConfig();
        }   
    }
}

void 
PinMcp::config( uint8_t direction, uint8_t pulldown, uint8_t pullup ) {
    if ( mPinManager ) {
        mPinManager->updateConfig();
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