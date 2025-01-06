#ifndef __VOLUME_H__
#define __VOLUME_H__

#include "../config.h"
#include <memory>

class Volume {
    public:
        virtual void mute( bool setMute ) = 0;
        virtual bool isMuted() const = 0;
        virtual int16_t getMaxAttenuation() const = 0;
        virtual int16_t getMinAttenuation() const = 0;
        virtual int16_t getAttenuationStep() const = 0;

        // Attenuation is in half a DB
        virtual void setAttenuation( uint16_t attenuation ) = 0; 
};  

typedef std::shared_ptr<Volume> VolumePtr;

#endif