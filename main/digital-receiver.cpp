#include "digital-receiver.h"
#include "debug.h"


DigitalReceiver::DigitalReceiver( I2CBUSPtr i2c, uint8_t addr, PinManagerPtr pinManager ) : mI2C( i2c ), mAddr( addr ), mCurrentInput( INPUT_NONE ) {
    mReset = pinManager->createPin( Pin::PIN_TYPE_OUTPUT, PinMcp::PIN_A7, Pin::PIN_TYPE_OUTPUT, Pin::PIN_PULLDOWN_DISABLE, Pin::PIN_PULLUP_ENABLE );
}

void 
DigitalReceiver::init() {
   mI2C->writeRegisterByte( mAddr, REGISTER_FORMAT, FORMAT_24B_I2S );

   // Set MPIO_A to be more SPDIF inputs
   mI2C->writeRegisterByte( mAddr, REGISTER_CONFIG, 0 );
}

void 
DigitalReceiver::setInput( uint8_t input ) {
    switch( input ) {
        case INPUT_SPDIF1:
            mI2C->writeRegisterByte( mAddr, REGISTER_INPUT, 0b01000000 );
            mI2C->writeRegisterByte( mAddr, REGISTER_OUTPUT_PORT, 0 );
            break;
        case INPUT_SPDIF2:
            mI2C->writeRegisterByte( mAddr, REGISTER_INPUT, 0b10000001 );    
            mI2C->writeRegisterByte( mAddr, REGISTER_OUTPUT_PORT, 0 );
            break;
        case INPUT_SPDIF3:
            mI2C->writeRegisterByte( mAddr, REGISTER_INPUT, 0b11000010 );   
            mI2C->writeRegisterByte( mAddr, REGISTER_OUTPUT_PORT, 0 );
            break;  
        case INPUT_SPDIF4:
            mI2C->writeRegisterByte( mAddr, REGISTER_INPUT, 0b11000011 );
            mI2C->writeRegisterByte( mAddr, REGISTER_OUTPUT_PORT, 0 );
            break; 
        case INPUT_I2S_1:
            mI2C->writeRegisterByte( mAddr, REGISTER_INPUT, 0b11000000 );
            mI2C->writeRegisterByte( mAddr, REGISTER_OUTPUT_PORT, 0b01000100 );
            break;
        case INPUT_I2S_2:
            // make sure coax amps are shut down
            mI2C->writeRegisterByte( mAddr, REGISTER_INPUT, 0b11000000 );
            mI2C->writeRegisterByte( mAddr, REGISTER_OUTPUT_PORT, 0b01010101 );
            break;
        default:
            AMP_DEBUG_E( "Unknown DigitalReceiver input" );
            break;
    }
}