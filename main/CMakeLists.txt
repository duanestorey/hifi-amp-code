# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS 
    "main.cpp" 
    "amplifier.cpp" 
    "wifi.cpp" 
    "timer.cpp" 
    "mutex.cpp" 
    "queue.cpp" 
    "i2c-bus.cpp" 
    "lcd.cpp" 
    "tmp100.cpp" 
    "encoder.cpp" 
    "button.cpp" 
    "http-server.cpp" 
    "debug.cpp" 
    "dac-pcm5142.cpp" 
    "ir-receiver.cpp" 
    "pin-esp32.cpp"
    "pin.cpp"
    "pin-mcp.cpp"
    "pin-mcp-manager.cpp"
    "volume-controller.cpp"
    "pin-manager.cpp"
    "mdns-net.cpp"
    "input.cpp"
    "state.cpp"
    "analog-channel-selector.cpp"
    "digital-receiver.cpp"
    "adc-pcm1863.cpp"
    "diagnostics.cpp"
)
set(COMPONENT_ADD_INCLUDEDIRS)

register_component()

spiffs_create_partition_image(files ../html FLASH_IN_PROJECT)


