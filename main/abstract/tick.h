#ifndef __TICK_H__
#define __TICK_H__

#include <memory>

class Tick {
    public:
        virtual void tick() = 0;
};

typedef std::shared_ptr<Tick> TickPtr;

#endif