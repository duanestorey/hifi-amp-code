#ifndef __INPUT_H__
#define __INPUT_H__

#include "config.h"
#include <stdlib.h>
#include <memory>
#include <string>

class Input {
    public:
        enum {
            INPUT_TYPE_UNKNOWN,
            INPUT_TYPE_ANALOG,
            INPUT_TYPE_DIGITAL
        };

        enum {
            INPUT_PORT_UNKNOWN,
            INPUT_PORT_RCA,
            INPUT_PORT_SPDIF,
            INPUT_PORT_TOSLINK
        };

        Input();
        Input( uint8_t audioType, uint8_t audioPort, uint8_t id, const std::string &name = "" );

        std::string mName;
        uint8_t mID;
        uint8_t mType;
        uint8_t mPort;
};

typedef std::shared_ptr<Input> InputPtr;

#endif