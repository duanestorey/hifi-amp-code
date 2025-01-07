#ifndef __PIN_H__
#define __PIN_H__

#include <driver/gpio.h>
#include <string>
#include <memory>
#include "../queue.h"

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
            PIN_STATE_LOW = 0,
            PIN_STATE_HIGH = 1
        };

        Pin( uint8_t direction, uint8_t pulldown, uint8_t pullup );
        virtual bool operator==( const Pin& a ) { return a.getClass() == getClass() && a.getPinID() == getPinID(); }
        virtual void config( uint8_t direction, uint8_t pulldown, uint8_t pullup ) = 0;
        virtual void enableInterrupt( uint8_t interruptType );

        virtual void setState( uint8_t state ) = 0;
        virtual uint8_t getState() const = 0;
        virtual uint8_t getDirection() const { return mDir; }
        virtual uint8_t getPullup() const { return mPullup; }
        virtual uint8_t getPulldown() const { return mPulldown; }
        
        void enable() { setState( PIN_STATE_HIGH ); }
        void disable() { setState( PIN_STATE_LOW ); }

        virtual std::string getClass() const = 0;
        virtual uint8_t getPinID() const = 0;
    protected:
        uint8_t mDir;
        uint8_t mPulldown;
        uint8_t mPullup;
        uint8_t mInterruptType;
};

typedef std::shared_ptr<Pin> PinPtr;

#endif