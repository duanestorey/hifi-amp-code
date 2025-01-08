#include "dac-pcm5142.h"
#include "debug.h"

DAC_PCM5142::DAC_PCM5142( uint8_t address, I2CBUSPtr bus ) : mAddress( address ), mI2C( bus ), mMuted( false ), mCurrentPage( 255 ), mPrecision( DAC::PRECISION_24_BIT ), 
    mFormat( DAC::FORMAT_I2S ), mDetectedSamplingRate( 0 ), mDetectedClkRatio( 0 ) {
}

DAC_PCM5142::~DAC_PCM5142() {
}

void
DAC_PCM5142::reset() {
    mCurrentPage = 255;
    switchToPage( 0 );

    // enter standby mode
    mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_PAGE_STANDBY, 16 );

    // reset all registers
    mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_PAGE_RESET, 0x11 );

    mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_PAGE_STANDBY, 0 );

    mCurrentPage = 255;
}

void 
DAC_PCM5142::init() {
    AMP_DEBUG_I( "Initializing" );

    reset();
    switchToPage( 0 );

    // switch from 8x interpolation to 16x, and enable double speed
    //mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_INT_SPEED, 0 | 16 );

    // set auto clock to on
    mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_AUTO_CLOCK, 0 );

    // Enable high attenuation filter
    mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_DSP, 0b00011 );

    // set left and right gain to 0dB
    switchToPage( 1 );
    mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_GAIN_CTRL, 0 );
    
}

void 
DAC_PCM5142::setFormat( uint8_t format ) {

    AMP_DEBUG_I( "Setting Format" );
    switchToPage( 0 );

    uint8_t precisionValue = 0;
    switch ( mPrecision ) {
        case DAC::PRECISION_16_BIT:
            AMP_DEBUG_I( "Starting DAC to 16 BIT" );
            precisionValue = 0b00;
            break;
        case DAC::PRECISION_24_BIT:
            AMP_DEBUG_I( "Starting DAC to 24 BIT" );
            precisionValue = 0b10;
            break;
        case DAC::PRECISION_32_BIT:
            AMP_DEBUG_I( "Starting DAC to 32 BIT" );
            precisionValue = 0b11;
            break;
        default:
            break;
    }

    switch( format ) {
        case DAC::FORMAT_I2S:
            // set I2S with 24 bits
            AMP_DEBUG_I( "Setting DAC format to I2S" );
            mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_FORMAT, precisionValue );
            mFormat = format;
            break;
        case DAC::FORMAT_LEFT_JUSTIFIED:    
            AMP_DEBUG_I( "Setting DAC format to Left Justified" );
            mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_FORMAT, 32 + 16 + precisionValue );
            mFormat = format;
            break;
            break;
        default:
            mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_FORMAT, precisionValue );
            break;
    }
}

void
DAC_PCM5142::detectAudio() {
    switchToPage( 0 );

    uint8_t audioInfo = 0;
    mI2C->readRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_GAIN_CTRL, audioInfo );

    mDetectedClkRatio = audioInfo & 0x0f;
    mDetectedSamplingRate = ( ( audioInfo & 0x70 ) >> 3 );
}

void 
DAC_PCM5142::enable( bool state ) {
    AMP_DEBUG_I( "Setting enable to %d", (int)state );
    switchToPage( 0 );

    if ( state ) {
        mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_PAGE_STANDBY, 0 );

        // let's see if we can figure out the audio
        detectAudio();
    } else {
        mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_PAGE_STANDBY, 16 );
    }
}

void 
DAC_PCM5142::mute( bool setMute ) {
    if ( setMute != mMuted ) {
        return;
    }

    switchToPage( 0 );
    if ( setMute ) {
        mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_PAGE_MUTE, 0x11 );
    } else {
        mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_PAGE_MUTE, 0x00 );
    }

    mMuted = setMute;
}

bool 
DAC_PCM5142::isMuted() const {
    return mMuted;
}


int16_t 
DAC_PCM5142::getMaxAttenuation() const {
    return 79;
}

int16_t 
DAC_PCM5142::getMinAttenuation() const {
    return 0;
}

int16_t 
DAC_PCM5142::getAttenuationStep() const {
    return 1;
}

void 
DAC_PCM5142::setAttenuation( uint16_t attenuation ) {
    _setAttenuation( attenuation );
}

void 
DAC_PCM5142::_setChannelAttenuation( int channel, int att ) {
    // 0db is       0b00110000
    // -103 db is   0b11111110 103db
    // mute is      0b11111111
    switchToPage( 0 );
    AMP_DEBUG_I( "Setting channel attenuation" );

    // value can only go up to 79
    uint8_t value = ((uint16_t)att) * 2;
    if ( value > 206 ) {
        value = 206;
    }

    value = 0b00110000 + value;
    if ( channel == DAC::FRONT_LEFT ) {
        mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_VOL_LEFT, value );
    } else if ( channel == DAC::FRONT_RIGHT ) {
        mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_REG_VOL_RIGHT, value );
    }
}

void 
DAC_PCM5142::_setAttenuation( int att ) {
    _setChannelAttenuation( DAC::FRONT_LEFT, att );
    _setChannelAttenuation( DAC::FRONT_RIGHT, att );
}

void 
DAC_PCM5142::switchToPage( uint8_t page ) {
    AMP_DEBUG_I( "Setting page to %d", (int)page );
    mI2C->writeRegisterByte( mAddress, DAC_PCM5142::PCM5142_PAGE_SELECT, page );

    mCurrentPage = page;
}

void 
DAC_PCM5142::setPrecision( uint8_t precision ) {
    if ( mPrecision <= 3 ) {
        mPrecision = precision;

        setFormat( mFormat );
    }
}

