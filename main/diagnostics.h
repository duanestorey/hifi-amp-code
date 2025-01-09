#ifndef __DIAGNOSTICS_H__
#define __DIAGNOSTICS_H__

#include <map>
#include <string>
#include <memory>
#include "abstract/temp-sensor.h"

class Diagnostics {
    public:
        Diagnostics();
        void addTemperatureSensor( const std::string &str, TempSensorPtr sensor );

        void dumpAllTemperatures();
        float getTemperature( const std::string &str );
    protected:
        std::map<std::string, TempSensorPtr> mSensors;
};

typedef std::shared_ptr<Diagnostics> DiagnosticsPtr;

#endif