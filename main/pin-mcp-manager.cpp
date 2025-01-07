#include "pin-mcp-manager.h"
#include <memory.h>

PinMcpManager::PinMcpManager( I2CBUSPtr bus, uint8_t addr, QueuePtr queue ) : mBus( bus ), mAddr( addr ), mInterruptQueue( queue ) {
}

uint8_t 
PinMcpManager::getState( uint8_t pin ) {
    uint8_t isSet = Pin::PIN_STATE_LOW;;
    uint8_t portState = 0;

    if ( isPortA( pin ) ) {
        mBus->readRegisterByte( mAddr, 0x12, portState );
    } else {
        mBus->readRegisterByte( mAddr, 0x13, portState );
    }
   
    if ( portState ) {
        if ( ( portState & ( 1 << ( ( pin & 0x0f ) - 1 ) ) ) > 0 ) {
            isSet = Pin::PIN_STATE_HIGH;
        }
    }

    return isSet;
}

void 
PinMcpManager::processPortAInterrupt() {
    uint8_t intState = 0;
    mBus->readRegisterByte( mAddr, 0x0e, intState );
    for (int i = 0 ; i < 8; i++ ) {
        if ( intState & ( 1 << i ) ) {
            mInterruptQueue->add( Message::MSG_GPIO_INTERRUPT, 1, i );
        }
    }
}
        
void 
PinMcpManager::processPortBInterrupt() {
    uint8_t intState = 0;
    mBus->readRegisterByte( mAddr, 0x0f, intState );
    for (int i = 0 ; i < 8; i++ ) {
        if ( intState & ( 1 << i ) ) {
            mInterruptQueue->add( Message::MSG_GPIO_INTERRUPT, 1, 0x10 | i );
        }
    }
}

void 
PinMcpManager::setState( uint8_t pin, uint8_t state ) {
    uint8_t portState = 0;

    if ( isPortA( pin ) ) {
        mBus->readRegisterByte( mAddr, 0x12, portState );
    } else {
        mBus->readRegisterByte( mAddr, 0x13, portState );
    }
   
    uint8_t pinValue = ( 1 << ( ( pin & 0x0f ) - 1 ) );

    if ( state == Pin::PIN_STATE_LOW ) {
        portState = portState & ~pinValue;
    } else if ( state == Pin::PIN_STATE_HIGH ) {
        portState = portState | pinValue;
    }

    if ( isPortA( pin ) ) {
        mBus->writeRegisterByte( mAddr, 0x12, portState );
    } else {
        mBus->writeRegisterByte( mAddr, 0x13, portState );
    }
}

bool 
PinMcpManager::isPortA( uint8_t pin ) {
    return ( pin & 0x10 );
}

void
PinMcpManager::updateConfig() {
    uint8_t portADir = 0xff;
    uint8_t portBDir = 0xff;
    uint8_t pullupA = 0x00;
    uint8_t pullupB = 0x00;

    for ( PinMap::iterator i = mPinMap.begin(); i != mPinMap.end(); i++ ) {
        if ( i->second->getDirection() == Pin::PIN_TYPE_OUTPUT ) {
            uint8_t pin = ( 1 << ( ( i->second->getPinID() & 0x0f ) - 1 ) );

            if ( isPortA( i->second->getPinID() ) ) {
                portADir = portADir & ~pin;

                if ( i->second->getPullup() == Pin::PIN_PULLUP_ENABLE ) {
                    pullupA = pullupA | pin;
                }
            } else {
                portBDir = portBDir & ~pin;

                if ( i->second->getPullup() == Pin::PIN_PULLUP_ENABLE ) {
                    pullupB = pullupB | pin;
                }
            }

            mBus->writeRegisterByte( mAddr, 0x00, portADir );
            mBus->writeRegisterByte( mAddr, 0x01, portBDir );
            mBus->writeRegisterByte( mAddr, 0x0c, pullupA );
            mBus->writeRegisterByte( mAddr, 0x0d, pullupB );
        }
    }
}

PinPtr 
PinMcpManager::createPin( uint8_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup ) {
    PinPtr newPtr = PinPtr( new PinMcp( this, pin, direction, pulldown, pullup ) );

    mPinMap[ pin ] = newPtr;

    return newPtr;
}

