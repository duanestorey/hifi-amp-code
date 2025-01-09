#include "diagnostics.h"
#include "debug.h"

Diagnostics::Diagnostics() {

}

void 
Diagnostics::addTemperatureSensor(  const std::string &str, TempSensorPtr sensor ) {
    mSensors[ str ] = sensor;
}

void 
Diagnostics::dumpAllTemperatures() {
    for ( std::map<std::string, TempSensorPtr>::iterator i = mSensors.begin(); i != mSensors.end(); i++ ) {
        AMP_DEBUG_I( "Temperature of [%s] is [%0.2f]", i->first.c_str(), i->second->readTemperature() );
    }
}

float 
Diagnostics::getTemperature( const std::string &str ) {
    float temp = 0;

    std::map<std::string, TempSensorPtr>::iterator i = mSensors.find( str );
    if ( i != mSensors.end() ) {
        temp = i->second->readTemperature();
    }

    return temp;
}