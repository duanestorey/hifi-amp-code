#ifndef __MESSAGE_H__
#define __MESSAGE_H__

#include <stdio.h>
#include <stdlib.h>

class Message {
public:
    Message() : mMessageType( MSG_NONE ), mParam( 0 ) {}

    enum MessageType {
        MSG_NONE = 0,
        MSG_TIMER,
        MSG_WIFI_INIT,
        MSG_WIFI_CONNECTED,
        MSG_AUDIO_SET_ENHANCEMENT,
        MSG_AUDIO_SHUTDOWN,
        MSG_AUDIO_RESTART,
        MSG_VOLUME_UP,
        MSG_VOLUME_DOWN,
        MSG_VOLUME_SET,
        MSG_DISPLAY_DONE_INIT,
        MSG_DISPLAY_SHOULD_UPDATE,
        MSG_WIFI_UPDATE,
        MSG_INPUT_UP,
        MSG_INPUT_DOWN,
        MSG_INPUT_SET,
        MSG_INPUT_BUTTON_PRESS,
        MSG_AUDIO_SAMPLING_RATE_CHANGE,
        MSG_BUTTON_PRESSED,
        MSG_BUTTON_RELEASED,
        MSG_POWEROFF,
        MSG_POWERON,
        MSG_IR_CODE_RECEIVER,
        MSG_GPIO_INTERRUPT,
        MSG_GPIO_MCP_PORTA,
        MSG_GPIO_MCP_PORTB
    };

    Message( MessageType t ) : mMessageType( t ), mParam( 0 ), mParam2( 0 ) {}
    Message( MessageType t, uint32_t param ) : mMessageType( t ), mParam( param ), mParam2( 0 ) {}
    Message( MessageType t, uint32_t param, uint32_t param2 ) : mMessageType( t ), mParam( param ), mParam2( param2 ) {}

    MessageType mMessageType;
    uint32_t mParam;
    uint32_t mParam2;
};

#endif