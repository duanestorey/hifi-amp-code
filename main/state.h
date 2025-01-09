#ifndef __STATE_H__
#define __STATE_H__

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "abstract/channel-sel.h"
#include "input.h"

class AmplifierState {
public:
    AmplifierState() : 
        mState( STATE_INIT ), 
        mSpeakerConfig( AUDIO_2_DOT_1 ), 
        mCurrentAttenuation( 30 ), 
        mSamplingRate( 48000 ), 
        mBitDepth( 24 ), 
        mConnected( false ), 
        mCurrentInput( 0 ) {}

    enum {
        STATE_INIT,
        STATE_PLAYING,
        STATE_MUTED,
        STATE_SLEEPING,
        STATE_UPDATING,
        STATE_ERROR_MINOR,
        STATE_ERROR_MAJOR
    };

    enum {
        AUDIO_2_CH,
        AUDIO_2_DOT_1
    };

    // Volume goes from 0-79
    uint8_t mState;
    uint8_t mSpeakerConfig;
    uint8_t mCurrentAttenuation;

    uint32_t mSamplingRate;
    uint8_t mBitDepth;

    bool mConnected;

    InputPtr mCurrentInput;

};

#endif