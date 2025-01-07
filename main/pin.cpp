#include "abstract/pin.h"

Pin::Pin( uint8_t direction, uint8_t pulldown, uint8_t pullup, uint8_t interrupt ) : mDir( direction ), mPulldown( pulldown ), mPullup( pullup ), mInt( interrupt ) {

}