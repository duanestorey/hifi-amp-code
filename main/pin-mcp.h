#ifndef __PINMCP_H__
#define __PINMCP_H__

#include "abstract/pin.h"

class PinMcpManager;

class PinMcp : public Pin {
    public:
        enum {
            PIN_A0 = 0x00,
            PIN_A1 = 0x01,
            PIN_A2 = 0x02,
            PIN_A3 = 0x03,
            PIN_A4 = 0x04,
            PIN_A5 = 0x05,
            PIN_A6 = 0x06,
            PIN_A7 = 0x07,
            PIN_B0 = 0x08,
            PIN_B1 = 0x09,
            PIN_B2 = 0x0a,
            PIN_B3 = 0x0b,
            PIN_B4 = 0x0c,
            PIN_B5 = 0x0d,
            PIN_B6 = 0x0e,
            PIN_B7 = 0x0f
        };

        PinMcp( PinMcpManager *pinManager, uint8_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt = PIN_INT_DISABLE );
        virtual void config( uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt = PIN_INT_DISABLE );
        virtual void setState( uint8_t state );

        virtual std::string getClass() const { return "MCP"; }
        virtual uint8_t getPinID() const { return mPin; }
        virtual uint8_t getState() const;
    protected:
        uint8_t mPin;
        PinMcpManager *mPinManager;
};

#endif