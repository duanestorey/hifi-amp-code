#include "mdns.h"
#include "mdns-net.h"
#include "debug.h"

MDNS::MDNS() {
    esp_err_t err = mdns_init();
    if ( err ) {
        AMP_DEBUG_E( "MDNS Init failed: %d\n", err );

    }
}

void
MDNS::start() {
    //set hostname
    mdns_hostname_set( "amp" ) ;
    mdns_instance_name_set( "Hifi Audio Amplifier" );
    mdns_service_add( NULL, "_http", "_tcp", 80, NULL, 0 );
}