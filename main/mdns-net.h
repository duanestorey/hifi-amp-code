#ifndef __MDNS_H__
#define __MDNS_H__

#include <memory>

class MDNS {
    public:
        MDNS();
        void start();
};

typedef std::shared_ptr<MDNS> MDNSPtr;

#endif