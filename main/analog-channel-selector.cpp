#include "analog-channel-selector.h"

AnalogChannelSelector::AnalogChannelSelector( I2CBUSPtr i2c, PinManagerPtr pinManager ) : mI2C( i2c ) {
    
}

void 
AnalogChannelSelector::selectChannel( uint8_t channel ) {

}