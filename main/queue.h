#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <stdio.h>
#include <memory>

#include "message.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define MESSAGE_QUEUE_SIZE 15

class Queue {
public:
    Queue( uint8_t queueSize = MESSAGE_QUEUE_SIZE );
    virtual ~Queue();

    void add( Message msg );
    void add( Message::MessageType msgType, uint32_t param = 0, uint32_t param2 = 0 ) { add( Message( msgType, param, param2 ) ); }

    void addFromISR( Message msg );
    void addFromISR( Message::MessageType msgType, uint32_t param = 0, uint32_t param2 = 0 ) { addFromISR( Message( msgType, param, param2 ) ); }

    bool hasMessage();
    Message getMessage();
    bool waitForMessage( Message &msg, uint32_t maxTimeInMS );
protected:
    QueueHandle_t mQueueHandle;
};

typedef std::shared_ptr<Queue> QueuePtr;

#endif