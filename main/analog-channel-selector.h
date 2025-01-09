#ifndef __ANALOG_CHANNEL_SELECTOR_H__
#define __ANALOG_CHANNEL_SELECTOR_H__

#include "button.h"
#include "pin-manager.h"
#include "i2c-bus.h"
#include "config.h"
#include <memory>

class AnalogChannelSelector {
    public:
        AnalogChannelSelector( I2CBUSPtr i2c, PinManagerPtr pinManager );
        void selectChannel( uint8_t channel );

    protected:
        ButtonPtr mEnable1;
        ButtonPtr mEnable2;
        ButtonPtr mEnable3;
        ButtonPtr mReset;

        I2CBUSPtr mI2C;
};

typedef std::shared_ptr<AnalogChannelSelector> AnalogChannelSelectorPtr;

#endif