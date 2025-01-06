#include "volume-controller.h"
#include <limits>

VolumeController::VolumeController() : mIsMuted( true ) {

}

bool 
VolumeController::isMuted() const {
    return mIsMuted;
}

int16_t 
VolumeController::getAttenuationStep() const {
    uint16_t attenuationStep = 1;

    for ( VolumeVector::const_iterator i = mVolumes.begin(); i != mVolumes.end(); i++ ) {
        attenuationStep = std::max<int16_t>( attenuationStep, (*i)->getAttenuationStep() );
    }

    return attenuationStep;
}

void
VolumeController::mute( bool setMute ) {
    for ( VolumeVector::const_iterator i = mVolumes.begin(); i != mVolumes.end(); i++ ) {
        (*i)->mute( setMute );
    }

    mIsMuted = setMute;
}

int16_t 
VolumeController::getMaxAttenuation() const {
    uint16_t maxAttenuation = 0;

    for ( VolumeVector::const_iterator i = mVolumes.begin(); i != mVolumes.end(); i++ ) {
        maxAttenuation = std::max<int16_t>( maxAttenuation, (*i)->getMaxAttenuation() );
    }

    return maxAttenuation;
}

int16_t 
VolumeController::getMinAttenuation() const {
    uint16_t minAttenuation = std::numeric_limits<int16_t>::max();

    for ( VolumeVector::const_iterator i = mVolumes.begin(); i != mVolumes.end(); i++ ) {
        minAttenuation = std::min<int16_t>( minAttenuation, (*i)->getMaxAttenuation() ); 
    }

    return minAttenuation;
}

void 
VolumeController::setAttenuation( uint16_t attenuation ) {
    for ( VolumeVector::const_iterator i = mVolumes.begin(); i != mVolumes.end(); i++ ) {
        uint16_t attenuationToSet = std::min<uint16_t>( attenuation, (*i)->getMaxAttenuation() ); 
        (*i)->setAttenuation( attenuationToSet );
    }
}

void 
VolumeController::addForControl( VolumePtr volume ) {
    mVolumes.push_back( volume );
    volume->mute( mIsMuted );
}