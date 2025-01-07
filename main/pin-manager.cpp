#include "pin-manager.h"
#include "pin-esp32.h"
#include "pin-mcp.h"
#include "debug.h"
#include "config.h"

void 
ESP32_GPIO_ISR( void *arg ) {
    ESP32_IntData *data = (ESP32_IntData *)arg;
    data->mPinManager->handleInterrupt( data->mPin );
}

void 
ESP32_GPIO_ISR_PORTA( void *arg ) {
    ((PinManager *)arg)->_handlePortA();
}

void 
ESP32_GPIO_ISR_PORTB( void *arg ) {
    ((PinManager *)arg)->_handlePortB();
}

PinManager::PinManager( I2CBUSPtr i2c, QueuePtr interruptQueue ) : mInterruptQueue( interruptQueue ), mI2C( i2c ) {
    mPinManagerMCP = PinMcpManagerPtr( new PinMcpManager( i2c, AMP_I2C_ADDR_MCP, interruptQueue ) );

    configureMCPInterrupts();
}

PinPtr 
PinManager::createPin( uint8_t pinType, uint8_t pinReference, uint8_t direction, uint8_t pulldown, uint8_t pullup ) {
    PinPtr pin;

    switch( pinType ) {
        case PIN_TYPE_ESP32:
            pin = PinPtr( new PinESP32( this, (gpio_num_t)pinReference, direction, pulldown, pullup ) ); 
            break;
        case PIN_TYPE_MCP:
            pin = PinPtr( new PinMcp( mPinManagerMCP.get(), pinReference, direction, pulldown, pullup ) );
            break;
        default:
            AMP_DEBUG_E( "Trying to create unknown pin type" );
            break;
    }

    return pin;
}

void 
PinManager::enableInterrupt( uint8_t pinReference, uint8_t interruptType ) {
    ESP32_IntData *pinData = new ESP32_IntData( pinReference, this );

    mInterrupts[ pinReference ] = pinData;

    switch( interruptType ) {
        case Pin::PIN_INT_LEADING:
            gpio_set_intr_type( (gpio_num_t)pinReference, GPIO_INTR_NEGEDGE );
            break;
        case Pin::PIN_INT_TRAILING:
            gpio_set_intr_type( (gpio_num_t)pinReference, GPIO_INTR_POSEDGE );
            break;
        case Pin::PIN_INT_BOTH:
            gpio_set_intr_type( (gpio_num_t)pinReference, GPIO_INTR_ANYEDGE );
            break;
    }
    
    gpio_isr_handler_add( (gpio_num_t)pinReference, ESP32_GPIO_ISR, (void *)pinData );
}

void 
PinManager::handleInterrupt( uint8_t pin ) {
    mInterruptQueue->addFromISR( Message::MSG_GPIO_INTERRUPT, (int32_t)PinManager::PIN_TYPE_ESP32, (int32_t)pin );
}

void 
PinManager::configureMCPInterrupts() {
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    // set up port a interrupt handler
    io_conf.pin_bit_mask = ( ((uint64_t)1) << ( uint64_t ) AMP_PIN_MCP_PORTA );
	gpio_config( &io_conf ); 
    gpio_isr_handler_add( (gpio_num_t)AMP_PIN_MCP_PORTA, ESP32_GPIO_ISR_PORTA, (void *)this );

    // set up port b interrupt handler
    io_conf.pin_bit_mask = ( ((uint64_t)1) << ( uint64_t ) AMP_PIN_MCP_PORTB );
	gpio_config( &io_conf );   
    gpio_isr_handler_add( (gpio_num_t)AMP_PIN_MCP_PORTB, ESP32_GPIO_ISR_PORTB, (void *)this );

    // set up reset button
    io_conf.pin_bit_mask = ( ((uint64_t)1) << ( uint64_t ) AMP_PIN_MCP_RESET );
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config( &io_conf );  

    // disable reset, enable MCP
    gpio_set_level( (gpio_num_t)AMP_PIN_MCP_RESET, 1 );
}

void 
PinManager::handleMcpPortA() {
    mPinManagerMCP->processPortAInterrupt();
}

void 
PinManager::handleMcpPortB() {
    mPinManagerMCP->processPortBInterrupt();
}

void 
PinManager::_handlePortA() {    
    mInterruptQueue->addFromISR( Message::MSG_GPIO_MCP_PORTA );
}

void 
PinManager::_handlePortB() {
    mInterruptQueue->addFromISR( Message::MSG_GPIO_MCP_PORTB ); 
}

