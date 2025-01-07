#include "pin-esp32.h"
#include "pin-manager.h"

PinESP32::PinESP32( PinManager *pinManager, gpio_num_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup ) : Pin( direction, pulldown, pullup ), mPinManager( pinManager ), mPin( pin ) {
    config( direction, pulldown, pullup );
}

void 
PinESP32::enableInterrupt( uint8_t interruptType ) {
    Pin::enableInterrupt( interruptType );

    gpio_int_type_t intType = GPIO_INTR_DISABLE;
    switch( interruptType ) {
        case Pin::PIN_INT_LEADING:
            intType = GPIO_INTR_POSEDGE;
            break;
        case Pin::PIN_INT_TRAILING:
            intType = GPIO_INTR_NEGEDGE;
            break;
        case Pin::PIN_INT_BOTH:
            intType = GPIO_INTR_ANYEDGE;
            break;
        default:
            break;

    }

    gpio_set_intr_type( mPin, intType );

    mPinManager->enableInterrupt( mPin, interruptType );

    gpio_intr_enable( mPin );
}

void 
PinESP32::config( uint8_t direction, uint8_t pulldown, uint8_t pullup ) {
    uint64_t mask = ( ((uint_fast32_t)1) << ( uint_fast32_t ) mPin );

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = mask;

    switch( direction ) {
        case Pin::PIN_TYPE_OUTPUT:
            io_conf.mode = GPIO_MODE_OUTPUT;
            break;
        case Pin::PIN_TYPE_INPUT:
            io_conf.mode = GPIO_MODE_INPUT;
            break;   
    }

    switch( pulldown ) {
        case Pin::PIN_PULLDOWN_ENABLE:
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;
        case Pin::PIN_PULLDOWN_DISABLE:
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;   
    }

    switch( pulldown ) {
        case Pin::PIN_PULLUP_ENABLE:
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            break;
        case Pin::PIN_PULLUP_DISABLE:
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            break;   
    }

	gpio_config( &io_conf );  
}

void 
PinESP32::setState( uint8_t state ) {
    if ( state == Pin::PIN_STATE_LOW ) {
        gpio_set_level( mPin, 0 );
    } else if ( state == Pin::PIN_STATE_HIGH ) {
        gpio_set_level( mPin, 1 );
    }
}

uint8_t 
PinESP32::getState() const {
    return gpio_get_level( mPin );
}

std::string 
PinESP32::getClass() const {
    return "ESP32";
}

uint8_t 
PinESP32::getPinID() const {
    return (uint8_t)mPin;
}