#ifndef __TMP100_H__
#define __TMP100_H__

#include "abstract/temp-sensor.h"
#include "i2c-bus.h"

class TMP100 : public TempSensor {
public:
    TMP100( uint8_t address, I2CBUSPtr bus );
    
    virtual float readTemperature();
protected:
    uint8_t mAddress;
    I2CBUSPtr mI2C;
};

#endif