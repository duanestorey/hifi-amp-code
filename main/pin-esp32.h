#ifndef __PINESP32_H__
#define __PINESP32_H__

#include "abstract/pin.h"

class PinESP32 : public Pin {
    public:
        PinESP32( gpio_num_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt );

        virtual void config( uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt = Pin::PIN_INT_DISABLE );
        virtual void setState( uint8_t state );
        virtual std::string getClass() const;
        virtual uint8_t getPinID() const;
        virtual uint8_t getState() const;
    protected:
        gpio_num_t mPin;
};

#endif