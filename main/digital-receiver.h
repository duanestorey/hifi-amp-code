#ifndef __DIGITAL_RECEIVER_H__
#define __DIGITAL_RECEIVER_H__

#include "config.h"
#include "i2c-bus.h"
#include "pin-manager.h"
#include <memory>

class DigitalReceiver {
    public:
        enum {
            INPUT_NONE,
            INPUT_SPDIF1,
            INPUT_SPDIF2,
            INPUT_SPDIF3,
            INPUT_SPDIF4,
            INPUT_I2S_1,
            INPUT_I2S_2
        };  

        enum {
            ERROR_FREQ_CHANGE = 0b100000,
            ERROR_NON_PCM = 0b001000,
            ERROR_VALID = 0xb000100,
            ERROR_PLL_UNLOCK = 0xb000001
        };

        enum {
            FORMAT_24B_I2S = 0b100,
            FORMAT_16B_RJ = 0b011,
            FORMAT_24B_LJ = 0b101
        };

        enum {
            REGISTER_ERROR = 0x25,
            REGISTER_INT0 = 0x2c,
            REGISTER_FORMAT = 0x2f,
            REGISTER_INPUT = 0x34,
            REGISTER_OUTPUT_PORT = 0x6b,
            REGISTER_CONFIG = 0x6f
        };

        DigitalReceiver( I2CBUSPtr i2c, uint8_t addr, PinManagerPtr pinManager );
        virtual void init();
        virtual void setInput( uint8_t input );

    protected: 
        I2CBUSPtr mI2C;
        
        uint8_t mAddr;
        uint8_t mCurrentInput;

        PinPtr mReset;
};

typedef std::shared_ptr<DigitalReceiver> DigitalReceiverPtr;

#endif