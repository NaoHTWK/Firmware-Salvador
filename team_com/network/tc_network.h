#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/vector.hpp>
#include <sstream>
#include <thread>

#include "tc_pub_sub.h"

class TeamComNetwork {
public:
    TeamComNetwork(uint8_t team_id, bool sending_allowed = true);
    ~TeamComNetwork() {
        shutdown = true;
        if (sender.joinable())
            sender.join();
        receiver.join();
    }

private:
    void send() {
        while (!shutdown) {
            TeamComData data = tc_internal::broadcast.next();
            std::string msg;
            std::stringstream ss;
            boost::archive::binary_oarchive oa(ss);
            oa << data;
            msg = ss.str();
            sendBroadcast(11000 + team_id, msg);
        }
    }
    void sendBroadcast(uint16_t port, const std::string& data);
    void receive();
    std::thread sender;
    std::thread receiver;
    uint8_t team_id;
    bool shutdown = false;
};
