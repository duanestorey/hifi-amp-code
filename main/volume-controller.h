#ifndef __VOLUME_CONTROLLER_H__
#define __VOLUME_CONTROLLER_H__

#include "abstract/volume.h"
#include <vector>
#include <memory>

typedef std::vector<VolumePtr> VolumeVector;

class VolumeController : public Volume {
    public:
        VolumeController();

        virtual bool isMuted() const;
        virtual void mute( bool setMute );
        virtual int16_t getMaxAttenuation() const;
        virtual int16_t getMinAttenuation() const;
        virtual void setAttenuation( uint16_t attenuation ); 
        virtual int16_t getAttenuationStep() const;
        virtual void addForControl( VolumePtr volume );
    protected:
        bool mIsMuted;
        VolumeVector mVolumes;
};

typedef std::shared_ptr<VolumeController> VolumeControllerPtr;

#endif