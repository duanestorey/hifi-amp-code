#include "input.h"

Input::Input() : mID( 0 ), mType( INPUT_TYPE_UNKNOWN ), mPort( INPUT_PORT_UNKNOWN ) {

}

Input::Input( uint8_t audioType, uint8_t audioPort, uint8_t id, const std::string &name ) : mName( name ), mID( id ), mType( audioType ), mPort( audioPort ) {

}