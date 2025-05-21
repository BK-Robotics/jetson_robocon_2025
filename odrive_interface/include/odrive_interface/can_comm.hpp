#ifndef CAN_COMM_HPP_
#define CAN_COMM_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <iostream>
#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <functional>

using FeedbackCallback = std::function<void(uint32_t frame_id, const uint8_t *data)>;

class CANInterface
{
public:
    CANInterface();
    ~CANInterface();

    // Open a CAN interface
    bool openInterface(const std::string &interface);

    // Send a CAN frame
    bool sendFrame(uint32_t frame_id, const std::vector<uint8_t> &data);

    // Register a callback for feedback
    void registerFeedbackCallback(FeedbackCallback cb);

    // Receive loop for incoming CAN frames
    void receiveLoop();

private:
    int can_socket_;
    FeedbackCallback callback_;
};

#endif // CAN_COMM_HPP_
