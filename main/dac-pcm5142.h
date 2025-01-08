#ifndef __DAC_PCM5142_H__
#define __DAC_PCM51X4_H__

#include "abstract/dac.h"
#include "abstract/volume.h"
#include "i2c-bus.h"

class DAC_PCM5142 : public DAC,
                    public Volume {
public:
    enum {
        PCM5142_PAGE_SELECT = 0,
        PCM5142_PAGE_RESET = 1,
        PCM5142_PAGE_STANDBY = 2,
        PCM5142_PAGE_MUTE = 3,
        PCM5142_REG_INT_SPEED = 34,
        PCM5142_REG_AUTO_CLOCK = 37,
        PCM5142_REG_FORMAT = 40,
        PCM5142_REG_DSP = 43,
        PCM5142_REG_VOL_LEFT = 61,
        PCM5142_REG_VOL_RIGHT = 62,
        PCM5142_REG_AUDIO_INFO = 91,
        PCM5142_REG_STATUS = 94,
        PCM5142_REG_SPEED_MONITOR = 115
    } PCM5142_PAGE_1;

    enum {
        PCM5142_REG_GAIN_CTRL = 2
        
    } PCM5142_PAGE_2;
    
    DAC_PCM5142( uint8_t address, I2CBUSPtr bus );
    virtual ~DAC_PCM5142();

	// the name for this DAC
	virtual std::string name() { return "PCM5142"; };
	virtual void init();
    virtual void setFormat( uint8_t format );
    virtual void enable( bool state );

    virtual void mute( bool setMute );
    virtual bool isMuted() const;
    virtual int16_t getMaxAttenuation() const;
    virtual int16_t getMinAttenuation() const;
    virtual int16_t getAttenuationStep() const;
    virtual void setAttenuation( uint16_t attenuation ); 

    virtual void setPrecision( uint8_t precision );
protected:
    uint8_t mAddress;
    I2CBUSPtr mI2C;
    bool mMuted;

    uint8_t mCurrentPage;
    uint8_t mPrecision;
    uint8_t mFormat;

    uint32_t mDetectedSamplingRate;
    uint16_t mDetectedClkRatio;
private:
    void switchToPage( uint8_t page );
    void reset();
    void detectAudio();

    void _setChannelAttenuation( int channel, int att );
	void _setAttenuation( int att );
};

#endif