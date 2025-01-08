#ifndef __TEMP_SENSOR_H__
#define __TEMP_SENSOR_H__

#include <memory>

class TempSensor {
    public:
        virtual float readTemperature() = 0;
};

typedef std::shared_ptr<TempSensor> TempSensorPtr;

#endif