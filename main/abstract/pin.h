#ifndef __PIN_H__
#define __PIN_H__

#include <driver/gpio.h>
#include <string>
#include <memory>

class Pin {
    public:     
        enum {
            PIN_TYPE_OUTPUT,
            PIN_TYPE_INPUT
        };

        enum {
            PIN_PULLDOWN_DISABLE,
            PIN_PULLDOWN_ENABLE
        };

        enum {
            PIN_PULLUP_DISABLE,
            PIN_PULLUP_ENABLE
        };

        enum {
            PIN_INT_DISABLE,
            PIN_INT_LEADING,
            PIN_INT_TRAILING,
            PIN_INT_BOTH
        };

        enum {
            PIN_STATE_LOW,
            PIN_STATE_HIGH
        };

        virtual bool operator==( const Pin& a ) { return a.getClass() == getClass() && a.getPinID() == getPinID(); }
        virtual void config( uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt = PIN_INT_DISABLE ) = 0;
        virtual void setState( uint8_t state ) = 0;
        virtual uint8_t getState() const = 0;
        virtual std::string getClass() const = 0;
        virtual uint8_t getPinID() const = 0;
    protected:
        uint8_t mType;
        uint8_t mPulldown;
        uint8_t mPullup;
        uint8_t mInt;
};

typedef std::shared_ptr<Pin> PinPtr;

#endif