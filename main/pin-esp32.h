#ifndef __PINESP32_H__
#define __PINESP32_H__

#include "abstract/pin.h"
#include "queue.h"

class PinManager;

class PinESP32 : public Pin {
    public:
        PinESP32( PinManager *pinManager, gpio_num_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup );

        virtual void config( uint8_t direction, uint8_t pulldown, uint8_t pullup );
        virtual void enableInterrupt( uint8_t interruptType );
        virtual void setState( uint8_t state );
        virtual std::string getClass() const;
        virtual uint8_t getPinID() const;
        virtual uint8_t getState() const;
    protected:
        PinManager *mPinManager; 
        gpio_num_t mPin;
};

#endif