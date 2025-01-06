#ifndef __I2CBUS_H__
#define __I2CBUS_H__

#include "mutex.h"
#include "config.h"
#include <memory>

class I2CBUS {
public:
    I2CBUS();

    bool writeRegisterByte( uint8_t address, uint8_t reg, uint8_t data );
    bool writeBytes( uint8_t address, uint8_t *data, uint8_t size );
    bool readRegisterByte( uint8_t address, uint8_t reg, uint8_t &data  );
    bool readRegisterBytes( uint8_t address, uint8_t reg, uint8_t dataSize, uint8_t *data  );
    void scanBus();
private:
    Mutex mMutex;
};

typedef std::shared_ptr<I2CBUS> I2CBUSPtr;

#endif