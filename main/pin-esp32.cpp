#include "pin-esp32.h"

PinESP32::PinESP32( gpio_num_t pin, uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt  ) : mPin( pin ) {
    config( direction, pulldown, pullup, interrupt );
}

void 
PinESP32::config( uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt ) {
    uint_fast32_t mask = ( ((uint_fast32_t)1) << ( uint_fast32_t ) mPin );

    gpio_config_t io_conf;
    io_conf.pin_bit_mask = mask;

    switch( interrupt ) {
        case Pin::PIN_INT_DISABLE:
            io_conf.intr_type = GPIO_INTR_DISABLE;
            break;
        case Pin::PIN_INT_LEADING:
            io_conf.intr_type = GPIO_INTR_POSEDGE;
            break;
        case Pin::PIN_INT_TRAILING:
            io_conf.intr_type = GPIO_INTR_NEGEDGE;
            break;
        case Pin::PIN_INT_BOTH:
            io_conf.intr_type = GPIO_INTR_ANYEDGE;
            break;
    }

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