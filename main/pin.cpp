#include "abstract/pin.h"

Pin::Pin( uint8_t direction, uint8_t pulldown, uint8_t pullup ) : mDir( direction ), mPulldown( pulldown ), mPullup( pullup ), mInterruptType( PIN_INT_DISABLE ) {

}

void 
Pin::enableInterrupt( uint8_t interruptType ) {
    mInterruptType = interruptType;
}