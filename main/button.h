#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "abstract/tick.h"
#include "queue.h"
#include <memory>

#define BUTTON_DEBOUNCE_TIME    25

class Button : public Tick {
public:
    Button( uint8_t gpio, QueuePtr queue, uint16_t debounceMs = BUTTON_DEBOUNCE_TIME );
    void handleInterrupt();
    
    virtual void tick();
protected:
    uint8_t mGpio;
    uint16_t mDebounceTime;

    QueuePtr mQueue;

    bool mThinking;
    uint32_t mThinkingStartTime;
    bool mPressed;
private:
    void startThinking();
    void stopThinking();
    void pressed();
    void unpressed();
};

typedef std::shared_ptr<Button> ButtonPtr;

#endif