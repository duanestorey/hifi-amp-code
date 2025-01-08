#ifndef __AMPLIFIER_H__
#define __AMPLIFIER_H__

#include "pins.h"
#include "timer.h"
#include "queue.h"

#include "state.h"
#include "mutex.h"
#include "i2c-bus.h"
#include "lcd.h"
#include "abstract/dac.h"
#include "abstract/channel-sel.h"

#include "tmp100.h"
#include "encoder.h"
#include "button.h"
#include "http-server.h"
#include "cs8416.h"
#include "driver/rmt_rx.h"
#include "pin-manager.h"
#include "abstract/pin.h"
#include "volume-controller.h"
#include "mdns-net.h"

class Amplifier {
public:
    Amplifier();
    void init();
    void handleDisplayThread();
    void handleAudioThread();
    void handleRadioThread();
    void handleAmplifierThread();

    void handleTimerThread();

    void _handleWifiCallback( int32_t event_id );
    void _handleDecoderISR();
    void _handlePowerButtonISR();
    void _handleVolumeButtonISR();
    void _handleVolumeButtonEncoderISR();
    void _handleInputButtonISR();
    void _handleInputButtonEncoderISR();
    void _handleNecRemoteCommand( uint8_t address, uint8_t command );

    void _handleIRInterrupt( const rmt_rx_done_event_data_t *edata );

    AmplifierState getCurrentState();
protected:
    void asyncUpdateDisplay();

    PinPtr mStandbyLED;

    bool mWifiEnabled;   
    bool mWifiConnectionAttempts;
    bool mUpdatingFromNTP;
    bool mPoweredOn;
    bool mDigitalAudioStarted;

    TimerPtr mTimer;

    QueuePtr mDisplayQueue;
    QueuePtr mAudioQueue;
    QueuePtr mRadioQueue;
    QueuePtr mAmplifierQueue;

    /*
    Queue mDisplayQueue;
    Queue mAudioQueue;
    Queue mRadioQueue;
    Queue mAmplifierQueue;
    */

    uint32_t mTimerID;
    uint32_t mButtonTimerID;
    uint32_t mReconnectTimerID;

    AmplifierState mState;
    I2CBUSPtr mI2C;

    LCDPtr mLCD;

    DACPtr mDAC[AMP_DAC_TOTAL_NUM];
    
    ChannelSel *mChannelSel;
    HTTPServerPtr mWebServer;
    MDNSPtr mDNS;

    CS8416 *mSPDIF;

    TempSensorPtr mMicroprocessorTemp;

    Encoder mVolumeEncoder;
    Encoder mInputEncoder;

    uint32_t mAudioTimerID;
    uint32_t mSpdifTimerID;
    
    bool mPendingVolumeChange;
    uint32_t mPendingVolume;

    ButtonPtr mPowerButton;
    ButtonPtr mVolumeButton;
    ButtonPtr mInputButton;

    uint8_t mIRBuffer[ 218 ];
    rmt_channel_handle_t mIRChannel;

    //PinMcpManagerPtr mMcpPinManager;
    PinManagerPtr mPinManager;

    VolumeControllerPtr mMasterVolume;

private:
    void setupWifi();
    void attemptWifiConnect();
    void taskDelayInMs( uint32_t ms );

    void updateTimeFromNTP();
    void handleWifiCallback( int32_t event_id );

    void updateInput( uint8_t newInput );
    void updateConnectedStatus( bool connected, bool doActualUpdate = true );

    void updateDisplay();

    void setupPWM();
    void handlePowerButtonPress();
    void handleInputButtonPress();
    void handleVolumeButtonPress();
    void handleDecoderIRQ();

    void changeAmplifierState( uint8_t newState );
    void startDigitalAudio();
    void stopDigitalAudio();

    void activateButtonLight( bool activate = true );
    void setupRemoteReceiver();
    void doActualRemoteReceive();

    Mutex mStateMutex;

};

#endif