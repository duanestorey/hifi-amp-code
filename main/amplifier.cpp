#include "amplifier.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "pins.h"
#include "esp_log.h"
#include "debug.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

#include "sdkconfig.h"
#include "dac-pcm5142.h"
#include "button.h"
#include "http-server.h"

#include "esp_netif_ip_addr.h"
#include "esp_mac.h"
#include "netdb.h"

Amplifier::Amplifier() : mWifiEnabled( false ), mWifiConnectionAttempts( 0 ), mUpdatingFromNTP( false ), mPoweredOn( true ), mTimerID( 0 ), mButtonTimerID( 0 ), mReconnectTimerID( 0 ),
    mCurrentInput( 0 ), mVolumeEncoder( 15, 13, true ), mInputEncoder( 4, 16, false ), mAudioTimerID( 0 ), mSpdifTimerID( 0 ), mPendingVolumeChange( false ), mPendingVolume( 0 ),
    mPowerButton( 0 ), mVolumeButton( 0 ), mInputButton( 0 ) {

}

void 
Amplifier::taskDelayInMs( uint32_t ms ) {
    vTaskDelay( ms / portTICK_PERIOD_MS );
}

AmplifierState 
Amplifier::getCurrentState() {
    ScopedLock lock( mStateMutex );

    AmplifierState stateCopy = mState;
    return stateCopy;
}

InputPtr 
Amplifier::getCurrentInput() {
    ScopedLock lock( mStateMutex );

    return mState.mCurrentInput;
}

void 
Amplifier::updateConnectedStatus( bool connected, bool doActualUpdate ) {
    {
        ScopedLock lock( mStateMutex );
        mState.mConnected = connected;
    }

    if ( doActualUpdate ) {
        asyncUpdateDisplay();
    }   

    if ( connected ) {
        mDNS->start();
        mWebServer->start();
    } else {
        mWebServer->stop();
    }
}

void
Amplifier::_handleIRInterrupt( const rmt_rx_done_event_data_t *edata ) {
    mAmplifierQueue->addFromISR( Message::MSG_IR_CODE_RECEIVER, (uint32_t)edata );
}

bool
rmt_rx_done_callback( rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx ) {
    (( Amplifier *)user_ctx)->_handleIRInterrupt( edata );
    return false;
}

void 
Amplifier::setupRemoteReceiver() {
    AMP_DEBUG_I( "Setting up IR receiver" );

    mIRChannel = NULL;

    rmt_rx_channel_config_t rx_chan_config;
    memset( &rx_chan_config, 0, sizeof( rx_chan_config ) );

    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;   // select source clock
    rx_chan_config.resolution_hz = 1 * 1000 * 1000; // 1 MHz tick resolution, i.e., 1 tick = 1 µs
    rx_chan_config.mem_block_symbols = 64;          // memory block size, 64 * 4 = 256 Bytes
    rx_chan_config.gpio_num = GPIO_NUM_26;              // GPIO number
    rx_chan_config.flags.invert_in = false;        // do not invert input signal
    rx_chan_config.flags.with_dma = false;          // do not need DMA backend

    ESP_ERROR_CHECK( rmt_new_rx_channel( &rx_chan_config, &mIRChannel ) );   

    rmt_rx_event_callbacks_t cbs = {
                .on_recv_done = rmt_rx_done_callback
            }; 

    ESP_ERROR_CHECK( rmt_rx_register_event_callbacks( mIRChannel, &cbs, this ) );

    rmt_enable( mIRChannel );

    doActualRemoteReceive();
}

// I2C current map
/*
    Current
    -------
    LCD                             0x27
    DIX9211                         0x40
    Microprocessor Temp Sensor      0x48   

    PCM 5142 FL/FR                  0x4c 

    Deprecated
    ------------
    Dolby Decoder STA310            0x60 
    PCM 1681 DAC                    0x4c
    Channel Selector AX2358         0x4a
    CS8416                          0x10

    Future  
    -------

*/

void 
Amplifier::init() {
    nvs_flash_init(); 

    // Create queues
    AMP_DEBUG_I( "Setting up message queues" );
    mDisplayQueue = QueuePtr( new Queue() );
    mAudioQueue = QueuePtr( new Queue() );
    mRadioQueue = QueuePtr( new Queue() );
    mAmplifierQueue = QueuePtr( new Queue() );

    // Create I2C bus
    AMP_DEBUG_I( "Setting up I2C bus" );
    mI2C = I2CBUSPtr( new I2CBUS() );

    // Create volume controller
    AMP_DEBUG_I( "Setting up volume controller" );
    mMasterVolume = VolumeControllerPtr( new VolumeController() );

    // Create pin manager
    AMP_DEBUG_I( "Setting up pin manager" );
    mPinManager = PinManagerPtr( new PinManager( mI2C, mAmplifierQueue ) );
    mStandbyLED = mPinManager->createPin( PinManager::PIN_TYPE_ESP32, AMP_PIN_STANDBY_LED, Pin::PIN_TYPE_OUTPUT, Pin::PIN_PULLDOWN_ENABLE, Pin::PIN_PULLUP_DISABLE );

    // Setup LCD display
    AMP_DEBUG_I( "Setting up LCD display" );
    mLCD = LCDPtr( new LCD( 0x27, mI2C ) );

    mDiagnostics = DiagnosticsPtr( new Diagnostics() );

    // Setup temperature sensors
    AMP_DEBUG_I( "Setting up temperature sensors" );

    mDiagnostics->addTemperatureSensor( "CPU", TempSensorPtr( new TMP100( 0x48, mI2C ) ) );
    mDiagnostics->addTemperatureSensor( "PSU", TempSensorPtr( new TMP100( 0x50, mI2C ) ) );
    mDiagnostics->addTemperatureSensor( "LEFT", TempSensorPtr( new TMP100( 0x49, mI2C ) ) );
    mDiagnostics->addTemperatureSensor( "RIGHT", TempSensorPtr( new TMP100( 0x50, mI2C ) ) );

    // setup channel selector
    AMP_DEBUG_I( "Setting up channel selectors" );
    mChannelSel = AnalogChannelSelectorPtr( new AnalogChannelSelector( mI2C, mPinManager ) );

    AMP_DEBUG_I( "Setting up digital receiver" );
    mDigitalReceiver = DigitalReceiverPtr( new DigitalReceiver( mI2C, AMP_I2C_ADDR_RECEIVER, mPinManager ) );

    // Setup output DACs
    AMP_DEBUG_I( "Setting up DACs" );
    for ( int i = 0; i < AMP_DAC_TOTAL_NUM; i++ ) {
        mDAC[i] = DACPtr( new DAC_PCM5142( 0x4c + i, mI2C ) );
        mDAC[i]->init();
        mDAC[i]->setFormat( DAC::FORMAT_I2S );

        mMasterVolume->addForControl( std::dynamic_pointer_cast<Volume>( mDAC[i] ) );
    }

    // Setup digital inputs
    AMP_DEBUG_I( "Setting up digital transceiver" );
    mDigitalReceiver->init();

    AMP_DEBUG_W( "TODO: Activate button light here" );

    // Set up buttons
    AMP_DEBUG_I( "Setting up button" );
    mPowerButton = ButtonPtr( new Button( PIN_BUTTON_POWER, mAmplifierQueue ) );
    mVolumeButton = ButtonPtr( new Button( PIN_BUTTON_VOLUME, mAmplifierQueue ) );
    mInputButton = ButtonPtr( new Button( PIN_BUTTON_INPUT, mAmplifierQueue ) );

    mNeedsTick.push_back( std::dynamic_pointer_cast<Tick>( mPowerButton ) );
    mNeedsTick.push_back( std::dynamic_pointer_cast<Tick>( mVolumeButton ) );
    mNeedsTick.push_back( std::dynamic_pointer_cast<Tick>( mInputButton ) );

    AMP_DEBUG_I( "Setting up web server" );
    mWebServer = HTTPServerPtr( new HTTPServer( mAmplifierQueue ) );

    // Setup timers
    AMP_DEBUG_I( "Setting up periodic timers" );
    mTimer = TimerPtr( new Timer() );
    mTimerID = mTimer->setTimer( 60000, mAmplifierQueue, true );
    mButtonTimerID = mTimer->setTimer( 10, mAmplifierQueue, true );
    mAudioTimerID = mTimer->setTimer( 1000, mAudioQueue, true );

    AMP_DEBUG_I( "Setting up IR receiver" );
    setupRemoteReceiver();

    // setting up inputs
    addInput( InputPtr( new Input( Input::INPUT_TYPE_DIGITAL, Input::INPUT_PORT_SPDIF, 0, "HDMI" ) ) );
    addInput( InputPtr( new Input( Input::INPUT_TYPE_DIGITAL, Input::INPUT_PORT_SPDIF, 1, "Blueray" ) ) );
    addInput( InputPtr( new Input( Input::INPUT_TYPE_DIGITAL, Input::INPUT_PORT_SPDIF, 2 ) ) );
    addInput( InputPtr( new Input( Input::INPUT_TYPE_DIGITAL, Input::INPUT_PORT_SPDIF, 3 ) ) );

    addInput( InputPtr( new Input( Input::INPUT_TYPE_ANALOG, Input::INPUT_PORT_RCA, 0 ) ) );
    addInput( InputPtr( new Input( Input::INPUT_TYPE_ANALOG, Input::INPUT_PORT_RCA, 1 ) ) );
    addInput( InputPtr( new Input( Input::INPUT_TYPE_ANALOG, Input::INPUT_PORT_RCA, 2 ) ) );
    addInput( InputPtr( new Input( Input::INPUT_TYPE_ANALOG, Input::INPUT_PORT_RCA, 3 ) ) );
    addInput( InputPtr( new Input( Input::INPUT_TYPE_ANALOG, Input::INPUT_PORT_RCA, 4 ) ) );
    addInput( InputPtr( new Input( Input::INPUT_TYPE_ANALOG, Input::INPUT_PORT_RCA, 5, "Vinyl" ) ) );
}

void 
Amplifier::asyncUpdateDisplay() {
    mDisplayQueue->add( Message::MSG_DISPLAY_SHOULD_UPDATE );
}

void 
Amplifier::handleDisplayThread() {
    AMP_DEBUG_I( "Starting Display Thread" );
    Message msg;

    // Startup
    mLCD->begin();

    // Let the audio task know the display is up and ready, should it care
    mAmplifierQueue->add( Message::MSG_DISPLAY_DONE_INIT );

    while( true ) {
        while ( mDisplayQueue->waitForMessage( msg, 10 ) ) {
            AMP_DEBUG_I( "Processing Display Queue Message" );

            switch( msg.mMessageType ) {
                case Message::MSG_DISPLAY_SHOULD_UPDATE:
                    updateDisplay();
                    break;
                default:
                    break;
            }
        } 
    } 
}

void
Amplifier::updateDisplay() {
    AmplifierState state = getCurrentState();

    char s[32];
    char d[14];

    sprintf( d, "Vol %ddB", ( -state.mCurrentAttenuation ) );

    if ( state.mConnected ) {
        sprintf( s, "%-12s%8s", d, "Wifi" );
    } else {
        sprintf( s, "%-20s", d );
    }
    mLCD->writeLine( 0, std::string( s ) );

    switch( state.mState ) {
        case AmplifierState::STATE_INIT:
            sprintf( s, "%-12s", "Starting");
            break;
        case AmplifierState::STATE_PLAYING:
            sprintf( s, "%-12s%7.1fC", "Playing", mDiagnostics->getTemperature( "CPU" ) );
            break;
        case AmplifierState::STATE_MUTED:
            sprintf( s, "%-12s", "Muted" );
            break;
        case AmplifierState::STATE_SLEEPING:
            sprintf( s, "%-12s", "Sleeping" );
            break;
        case AmplifierState::STATE_UPDATING:
            sprintf( s, "%-12s", "Updating" );
            break;
        case AmplifierState::STATE_ERROR_MINOR:
            sprintf( s, "%-12s", "Minor Error" );
            break;
        case AmplifierState::STATE_ERROR_MAJOR:
            sprintf( s, "%-12s", "Major Error" );
            break;
    }
    mLCD->writeLine( 1, s );

    char rate[15] = {0};
    switch( state.mCurrentInput->mType ) {
        case Input::INPUT_TYPE_ANALOG:
            sprintf( s, "Analog" );
            break;
        case Input::INPUT_TYPE_DIGITAL:
            if ( state.mSamplingRate == 44100 ) {
                sprintf( rate, "44.1k/%d", state.mBitDepth );
            } else {
                sprintf( rate, "%luk/%d", state.mSamplingRate / 1000, state.mBitDepth );
            }   

            sprintf( s, "%-10s%10s", "Digital", rate );
            break;
    }

    mLCD->writeLine( 2, std::string( s ) );

    char input[24];
    char audioType[10];
    switch( state.mSpeakerConfig ) {
        case AmplifierState::AUDIO_2_CH:
            sprintf( audioType, "2.0" );
            break;
        case AmplifierState::AUDIO_2_DOT_1:
            strcpy( audioType, "2.1" );
            break;
    }

    sprintf( input, "%-12s%8s", state.mCurrentInput->mName.c_str(), audioType );
    mLCD->writeLine( 3, input );
}

void 
Amplifier::changeAmplifierState( uint8_t newState ) {
    {
        ScopedLock lock( mStateMutex );
        mState.mState = newState;
    }

    asyncUpdateDisplay();
}

void 
Amplifier::handleTimerThread() {
    AMP_DEBUG_I( "Starting Timer Thread on Core %d", xPortGetCoreID() );
    while( true ) {
        mTimer->processTick();

        taskDelayInMs( 5 );
    } 
}

void 
Amplifier::doActualRemoteReceive() {       
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560 µs, 1250 ns < 560 µs, valid signal is not treated as noise
        .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000 µs, 12000000 ns > 9000 µs, the receive does not stop early
    };

    rmt_receive( mIRChannel, &mIRBuffer[ 0 ], sizeof( mIRBuffer ), &receive_config );               
}

void 
Amplifier::_handleNecRemoteCommand( uint8_t address, uint8_t command ) {
    AMP_DEBUG_I( "Handling NEC Remote addr-command [%d %d]", address, command );
    switch( address ) {
        case 4:
            switch( command ) {
                case 2: 
                    // volume up
                    mAmplifierQueue->add( Message::MSG_VOLUME_UP );
                    break;
                case 3:
                    // volume down
                    mAmplifierQueue->add( Message::MSG_VOLUME_DOWN );
                    break;
            }
            break;
    }
}

void 
Amplifier::handleAmplifierThread() {
    AMP_DEBUG_I( "Starting Amplifier Thread on Core %lu",( uint32_t )xPortGetCoreID() );

    Message msg;
    while( true ) {
        while ( mAmplifierQueue->waitForMessage( msg, 20 ) ) {
           // AMP_DEBUG_I( "Processing Amplifier Queue Message" );

            switch( msg.mMessageType ) {
                case Message::MSG_IR_CODE_RECEIVER:  
                    {
                        const rmt_rx_done_event_data_t *edata = (rmt_rx_done_event_data_t *)msg.mParam;
                        if ( edata->num_symbols == 34 ) {
                            uint8_t data[4] = {0};
                            uint8_t pos = 0;

                            for ( uint8_t i = 0; i < 4 ; i++ ) {
                                for( uint8_t x = 0 ; x < 8; x++ ) {
                                   // AMP_DEBUG_I( "%02d 0: ", edata->received_symbols[ pos + 1 ].duration0 );
                                   // AMP_DEBUG_I( "%02d 1: ", edata->received_symbols[ pos + 1 ].duration1 );

                                    if ( edata->received_symbols[ pos + 1 ].duration1 > 700 ) {
                                        // 1
                                        data[ i ] = ( data[ i ] >> 1 ) | 0x80;
                                    } else {
                                        // 0
                                        data[ i ] = data [ i ] >> 1;
                                    }
                                    pos++;
                                }
                            }

                            AMP_DEBUG_I( "Decoded %#02x %#02x %#02x %#02x", data[ 0 ], data[ 1 ] , data[ 2 ], data[ 3 ] );
                            AMP_DEBUG_I( "Decoded %#02x %#02x %#02x %#02x", data[ 0 ], (uint8_t)~data[ 1 ] , data[ 2 ], (uint8_t)~data[ 3 ] );

                            if ( data[ 0 ] == (uint8_t)~data[ 1 ] && data[ 2 ] == (uint8_t)~data[ 3 ] ) {
                                AMP_DEBUG_I( "Valid address %0x, valid command %0x", data[ 0 ], data[ 2 ] );

                                _handleNecRemoteCommand( data[ 0 ], data[ 2 ] );
                            }
                        }

                        AMP_DEBUG_I( "IR remote control code received [%d]", edata->num_symbols );
                        doActualRemoteReceive();
                    }
                    break;
                case Message::MSG_POWEROFF:
                    activateButtonLight( false );

                    mPoweredOn = false;

                    gpio_set_level( PIN_RELAY, 0 );
                    mLCD->enableBacklight( false );
                    mAudioQueue->add( Message::MSG_AUDIO_SHUTDOWN );

                    break;
                case Message::MSG_POWERON:
                    activateButtonLight( true ); 

                    mPoweredOn = true;

                    mLCD->enableBacklight( true );
                    gpio_set_level( PIN_RELAY, 1 );
                    taskDelayInMs( 1000 );

                    mAudioQueue->add( Message::MSG_AUDIO_RESTART );
                    
                    break;
                case Message::MSG_DISPLAY_SHOULD_UPDATE:
                    // need to update the display
                    break;
                case Message::MSG_DISPLAY_DONE_INIT:
                    // We are done display startup, let's start wifi now
                    mRadioQueue->add( Message::MSG_WIFI_INIT );
                    break;
                case Message::MSG_TIMER:
                    if ( msg.mParam == mTimerID ) {
                        AMP_DEBUG_I( "In Periodic Timer Event, Temp is %0.2f", mDiagnostics->getTemperature( "CPU" ) );
                        asyncUpdateDisplay();
                    } else if ( msg.mParam == mButtonTimerID ) {
                        // Process all button ticks
                        for ( std::vector<TickPtr>::iterator i = mNeedsTick.begin(); i != mNeedsTick.end(); i++ ) {
                            (*i)->tick();
                        }
                    }
                    break;  
                case Message::MSG_VOLUME_UP:
                    {
                        ScopedLock lock( mStateMutex );

                        uint8_t increase = msg.mParam;
                        if ( increase == 0 ) increase = 1;

                        if ( ( mState.mCurrentAttenuation - increase ) > 0 ) {
                            mState.mCurrentAttenuation = mState.mCurrentAttenuation - increase;
                        } else {
                            mState.mCurrentAttenuation = 0;
                        }
                        
                        asyncUpdateDisplay();

                        mAudioQueue->add( Message::MSG_VOLUME_SET, mState.mCurrentAttenuation );
                    }
                    break;
                case Message::MSG_VOLUME_DOWN:
                    {
                        ScopedLock lock( mStateMutex );

                        uint8_t decrease = msg.mParam;
                        if ( decrease == 0 ) decrease = 1;

                        if ( ( mState.mCurrentAttenuation + decrease ) <= 79 ) {
                            mState.mCurrentAttenuation = mState.mCurrentAttenuation + decrease;
                        } else {
                            mState.mCurrentAttenuation = 79;
                        }

                        asyncUpdateDisplay();

                        mAudioQueue->add( Message::MSG_VOLUME_SET, mState.mCurrentAttenuation );
                    }  
                    break;
                case Message::MSG_INPUT_UP:
                    {
                        if ( mCurrentInput < ( mAllInputs.size() - 1 ) ) {
                            mCurrentInput++;
                        } else {
                            mCurrentInput = 0;
                        }

                        ScopedLock lock( mStateMutex );
                        mState.mCurrentInput = mAllInputs[ mCurrentInput ];

                        mAudioQueue->add( Message::MSG_INPUT_SET );
                        asyncUpdateDisplay();
                    }

                    break;
                case Message::MSG_INPUT_DOWN:
                    {
                        if ( mCurrentInput > 0 ) {
                            mCurrentInput--;
                        } else {
                            mCurrentInput = mAllInputs.size() - 1;
                        }

                        ScopedLock lock( mStateMutex );
                        mState.mCurrentInput = mAllInputs[ mCurrentInput ];

                        mAudioQueue->add( Message::MSG_INPUT_SET );
                        asyncUpdateDisplay();
                    }
                
                    break;
                case Message::MSG_BUTTON_PRESSED:
                    AMP_DEBUG_I(  "Button pressed! %lu", msg.mParam );
                    switch( msg.mParam ) {
                        case PIN_BUTTON_POWER:
                            handlePowerButtonPress();
                            break;
                        case PIN_BUTTON_VOLUME:
                            handleVolumeButtonPress();
                            break;
                        case PIN_BUTTON_INPUT:
                            handleInputButtonPress();
                            break;
                        default:
                            break;
                    }
                    break;  
                case Message::MSG_BUTTON_RELEASED:
                    AMP_DEBUG_I(  "Button released! %lu", msg.mParam );    
                    break;          
                default:
                    break;
            }
        } 
    } 
}   

void
Amplifier::handlePowerButtonPress() {
    AMP_DEBUG_I( "Power button pressed" );

    if ( mPoweredOn ) {
        mAmplifierQueue->add( Message::MSG_POWEROFF );
    } else {
        mAmplifierQueue->add( Message::MSG_POWERON );
    }
}

void 
Amplifier::handleInputButtonPress() {
    AMP_DEBUG_I( "Input button pressed" );
}

void 
Amplifier::handleVolumeButtonPress() {
    AMP_DEBUG_I( "Volume button pressed" );
}

void 
Amplifier::audioChangeInput() {
    // change the input to what is specified in the configuration
    InputPtr currentInput = getCurrentInput();
    AMP_DEBUG_I( "Attemping to change audio input" );

    if ( currentInput.get() ) {
        if( currentInput->mType == Input::INPUT_TYPE_ANALOG ) {
            // analog input
            if ( currentInput->mPort == Input::INPUT_PORT_RCA ) {
                AMP_DEBUG_I( "...Setting RCA input to %d", mState.mCurrentInput->mID );
                // rca input
                mChannelSel->selectChannel( currentInput->mID + 1 );
            }
        } else if ( currentInput->mType == Input::INPUT_TYPE_DIGITAL ) {
            // deselect all relays to save power
            mChannelSel->selectChannel( 0 );
            
            if ( currentInput->mPort == Input::INPUT_PORT_SPDIF ) {
                AMP_DEBUG_I( "...Setting SPDIF input to %d", currentInput->mID );

                mDigitalReceiver->setInput( currentInput->mID );
            }
        }
    }
}

void 
Amplifier::startAudio() {
    AMP_DEBUG_I( "Starting audio" );

    mSpdifTimerID = mTimer->setTimer( 1000, mAudioQueue, true );
    
    // set the proper input channel
    AMP_DEBUG_I( "Restoring inputs" );
    audioChangeInput();

    AMP_DEBUG_I( "Setting previous volume" );    
    mMasterVolume->setAttenuation( mState.mCurrentAttenuation );

    AMP_DEBUG_I( "Enabling DACs" );
    for ( int i = 0; i < AMP_DAC_TOTAL_NUM; i++ ) {
        mDAC[i]->enable( true );
    }

    AMP_DEBUG_I( "Unmuting outputs" );
    mMasterVolume->mute( false );

    AMP_DEBUG_I( "Switching to PLAY state" );
    changeAmplifierState( AmplifierState::STATE_PLAYING );

    asyncUpdateDisplay();

}

void 
Amplifier::stopAudio() {
    AMP_DEBUG_I( "Stopping Audio" );

    mTimer->cancelTimer( mSpdifTimerID );
    mSpdifTimerID = 0;

    AMP_DEBUG_I( "Muting volume" );
    mMasterVolume->mute( true );

    // Disable the DACs
    AMP_DEBUG_I( "Disabling DACs" );
    for ( int i = 0; i < AMP_DAC_TOTAL_NUM; i++ ) {
        mDAC[i]->enable( false );
    }

    AMP_DEBUG_I( "Switching to SLEEP state" );
    changeAmplifierState( AmplifierState::STATE_SLEEPING );
}

void 
Amplifier::handleAudioThread() {
    AMP_DEBUG_I( "Starting Audio Thread" );
    
    mI2C->scanBus();

    // Set the initial channel input to the first slot
    {
        ScopedLock lock( mStateMutex );

        mCurrentInput = 0;
        mState.mCurrentInput = mAllInputs[ mCurrentInput ];
    }

    startAudio();

    Message msg;
    while( true ) {
        while ( mAudioQueue->waitForMessage( msg, 5 ) ) {
            switch( msg.mMessageType ) {
                case Message::MSG_AUDIO_RESTART:
                    startAudio();
                    break;
                case Message::MSG_AUDIO_SHUTDOWN:
                    stopAudio();
                    break;
                case Message::MSG_VOLUME_SET:
                    AMP_DEBUG_I( "Setting pending audio volume to %lu", msg.mParam );
                    mPendingVolumeChange = true;
                    mPendingVolume = msg.mParam;
                    
                    break;
                case Message::MSG_INPUT_SET:
                    AMP_DEBUG_I( "Setting audio input" );

                    AMP_DEBUG_E( "Need to do actual audio input selection here" );

                    //mChannelSel->setInput( ChannelSel::INPUT_STEREO_1 );
                    break;
                case Message::MSG_TIMER: 
                    if ( msg.mParam == mSpdifTimerID ) {
                               
                    } else if ( msg.mParam == mAudioTimerID ) {
                        // we are on the audio timer thread
                        if ( mPendingVolumeChange ) {
                            AMP_DEBUG_I( "Actually setting pending audio volume to %lu", mPendingVolume );

                            mMasterVolume->setAttenuation( mPendingVolume );
                        }
                    } 
                    
                    break;
                case Message::MSG_AUDIO_SAMPLING_RATE_CHANGE:
                    {
                        ScopedLock lock( mStateMutex );
                        mState.mSamplingRate = msg.mParam;
                    }

                    asyncUpdateDisplay();
                    break;
                default:
                    break;
            }
        } 
    } 
}

static void wifi_event_handler( void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data ){
    (( Amplifier *)event_handler_arg)->_handleWifiCallback( event_id );
}

void
Amplifier::_handleWifiCallback( int32_t event_id ) {
    mRadioQueue->add( Message::MSG_WIFI_UPDATE, event_id );
}

void
Amplifier::handleWifiCallback( int32_t event_id ) {
    AMP_DEBUG_I( "Processing Wifi Callback" ); 
    switch( event_id ) {
        case WIFI_EVENT_STA_START:
            AMP_DEBUG_I( "Wifi Starting Up" );
            break;
        case WIFI_EVENT_STA_CONNECTED:
            AMP_DEBUG_I( "Wifi Connection Established" );

            updateConnectedStatus( true );
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            AMP_DEBUG_I( "Wifi Disconnected" );
            if ( mWifiEnabled ) {
                AMP_DEBUG_I( "Setting Reconnection Timer" );
                mReconnectTimerID = mTimer->setTimer( 10000, mRadioQueue, false );
            }
            updateConnectedStatus( false );
            break;
        case IP_EVENT_STA_GOT_IP:
            AMP_DEBUG_I( "Wifi Received IP Address" );
            updateTimeFromNTP();
            break;
    }
}

void NTP_Callback(struct timeval *tv) {
    AMP_DEBUG_I( "Received NTP Callback" );  
}

 void 
 Amplifier::updateTimeFromNTP() {
    AMP_DEBUG_I( "Attemping To Update Time From NTP Server" );
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG( "pool.ntp.org" );

    esp_netif_sntp_init( &config );
    sntp_set_time_sync_notification_cb( &NTP_Callback );

    setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
    tzset();

    mUpdatingFromNTP = true;
    esp_netif_sntp_start();
 }

void 
Amplifier::setupWifi() {
    esp_netif_init();
    esp_event_loop_create_default(); 
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, (void*)this );
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, (void*)this );//creating event handler register for ip event
    wifi_config_t wifi_config;
    memset( &wifi_config, 0, sizeof( wifi_config ) );

    strcpy( (char*)wifi_config.sta.ssid, (char*)"The Grey Havens (LR)" );
    strcpy( (char*)wifi_config.sta.password, (char*)"brazil1234!" );

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config( WIFI_IF_STA, &wifi_config );

    mWifiEnabled = true;

    mDNS = MDNSPtr( new MDNS() );

    esp_wifi_start();
}

void 
Amplifier::attemptWifiConnect() {
    AMP_DEBUG_I( "Attempting to connect Wifi" );

    mWifiConnectionAttempts = 0;

    AMP_DEBUG_I( "Calling wifi connect" );
    esp_wifi_connect();
    AMP_DEBUG_I( "Wifi connect done" );
}

void
Amplifier::handleRadioThread() {
    AMP_DEBUG_I( "Starting Radio Thread on Core %lu", (uint32_t)xPortGetCoreID() );

    setupWifi();
    attemptWifiConnect();

    Message msg;
    while( true ) {
        if ( mUpdatingFromNTP ) {
            if ( SNTP_SYNC_STATUS_COMPLETED == sntp_get_sync_status() ) {
                mUpdatingFromNTP = false;

                time_t now;
                struct tm timeinfo;
                char strftime_buf[64];

                time(&now);
                localtime_r(&now, &timeinfo);
                strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                
                AMP_DEBUG_I("The current date/time is: %s", strftime_buf  );  

                esp_netif_sntp_deinit();
            }
        }

        while ( mRadioQueue->waitForMessage( msg, 10 ) ) {
            AMP_DEBUG_I( "Processing Radio Queue Message" );
            switch( msg.mMessageType ) {
                case Message::MSG_WIFI_INIT:
                   
                    break;
                case Message::MSG_WIFI_UPDATE:
                    handleWifiCallback( msg.mParam );
                    break;
                case Message::MSG_TIMER:
                    if ( msg.mParam == mReconnectTimerID ) {
                        AMP_DEBUG_I( "In Reconnection Timer Event" );
                        attemptWifiConnect();
                        mReconnectTimerID = 0;
                    } else if ( msg.mParam == mTimerID ) {
                        AMP_DEBUG_I( "In Periodic Timer Event" );
                    } else {
                        AMP_DEBUG_I( "Unknown timer event %lu", msg.mParam );
                    }
                   
                    break;
                default:
                    break;
            }
        } 
    } 
}

void 
Power_Button_ISR( void *arg ) {
    (( Amplifier * )arg)->_handlePowerButtonISR();
}

void 
Volume_Button_ISR( void *arg ) {
    (( Amplifier * )arg)->_handleVolumeButtonISR();
}

void 
Volume_Button_Encoder_ISR( void *arg ) {
    (( Amplifier * )arg)->_handleVolumeButtonEncoderISR();
}

void 
Input_Button_ISR( void *arg ) {
    (( Amplifier * )arg)->_handleInputButtonISR();
}

void
Input_Button_Encoder_ISR( void *arg ) {
    (( Amplifier * )arg)->_handleInputButtonEncoderISR();
}

void 
Amplifier::_handlePowerButtonISR() {
    if ( mPowerButton ) {
        mPowerButton->handleInterrupt();
    }
}

void
Amplifier::_handleVolumeButtonISR() {
   if ( mVolumeButton ) {
        mVolumeButton->handleInterrupt();
    }
}

void 
Amplifier::_handleInputButtonISR() {
   if ( mInputButton ) {
        mInputButton->handleInterrupt();
    }
}

void 
Amplifier::_handleVolumeButtonEncoderISR() {
    ENCODER_DIR direction = mVolumeEncoder.process();
    switch( direction ) {
        case ENCODER_FORWARD:
            mAmplifierQueue->addFromISR( Message::MSG_VOLUME_UP );
            break;
        case ENCODER_BACKWARD:
            mAmplifierQueue->addFromISR( Message::MSG_VOLUME_DOWN );
            break;
        default:
            break;
    }
}

void 
Amplifier::_handleInputButtonEncoderISR() {
    ENCODER_DIR direction = mInputEncoder.process();
    switch( direction ) {
        case ENCODER_FORWARD:
            mAmplifierQueue->addFromISR( Message::MSG_INPUT_UP );
            break;
        case ENCODER_BACKWARD:
            mAmplifierQueue->addFromISR( Message::MSG_INPUT_DOWN );
            break;
        default:
            break;
    }
}

void 
Amplifier::setupPWM() {
    ledc_timer_config_t backlight_config = {};
    backlight_config.speed_mode         = LEDC_LOW_SPEED_MODE;
    backlight_config.duty_resolution    = LEDC_TIMER_10_BIT;
    backlight_config.timer_num          = LEDC_TIMER_0;
    backlight_config.freq_hz            = 200;
    backlight_config.clk_cfg            = LEDC_USE_RC_FAST_CLK;

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num   = PIN_LED_FRONT_STANDBY;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_0;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel  = LEDC_TIMER_0;
    ledc_channel.duty       = 0;
    ledc_channel.hpoint     = 0;
    ledc_channel.flags.output_invert = 0;

    ledc_timer_config( &backlight_config );
    ledc_channel_config( &ledc_channel );
    
    ledc_set_duty( LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 50 );
	ledc_update_duty( LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0 );

    ledc_channel.gpio_num   = PIN_LED_ACTIVE;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel  = LEDC_TIMER_0;
    ledc_channel.duty       = 0;
    ledc_channel.hpoint     = 0;
    ledc_channel.flags.output_invert = 0;

    ledc_channel_config( &ledc_channel );

    ledc_set_duty( LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1023 );
	ledc_update_duty( LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 );
}

void 
Amplifier::activateButtonLight( bool activate ) {
    if ( activate ) {
        ledc_set_duty( LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 700 );
	    ledc_update_duty( LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 );
    } else {
        ledc_set_duty( LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0 );
	    ledc_update_duty( LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 );
    }

}