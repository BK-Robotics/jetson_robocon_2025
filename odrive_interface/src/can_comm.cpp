#include "odrive_interface/can_comm.hpp"

CANInterface::CANInterface() : can_socket_(-1) {}

CANInterface::~CANInterface()
{
    if (can_socket_ >= 0)
    {
        close(can_socket_);
    }
}

bool CANInterface::openInterface(const std::string &interface)
{
    if (can_socket_ >= 0)
    {
        close(can_socket_);
    }

    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
        std::cerr << "Error: could not create CAN socket." << std::endl;
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        std::cerr << "Error: could not get interface index for " << interface << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        std::cerr << "Error: could not bind socket to interface " << interface << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    int recv_own = 1;
    if (setsockopt(can_socket_, SOL_CAN_RAW,
                   CAN_RAW_RECV_OWN_MSGS,
                   &recv_own, sizeof(recv_own)) < 0)
    {
        std::cerr << "Warning: cannot enable CAN_RAW_RECV_OWN_MSGS\n";
    }

    std::cout << "Socket successfully bound to " << interface << std::endl;
    return true;
}

bool CANInterface::sendFrame(uint32_t frame_id, const std::vector<uint8_t> &data)
{
    if (can_socket_ < 0)
    {
        std::cerr << "Error: Socket is not open. Call openInterface() first." << std::endl;
        return false;
    }

    if (data.size() > 8)
    {
        std::cerr << "Error: CAN frame data length exceeds 8 bytes." << std::endl;
        return false;
    }

    struct can_frame frame{};
    frame.can_id = frame_id & CAN_SFF_MASK;
    frame.can_dlc = data.size();
    std::memset(frame.data, 0, sizeof(frame.data));
    std::memcpy(frame.data, data.data(), data.size());

    int nbytes = write(can_socket_, &frame, sizeof(frame));
    if (nbytes != static_cast<int>(sizeof(frame)))
    {
        std::cerr << "Error: Failed to send CAN frame." << std::endl;
        return false;
    }

    std::cout << "Sent CAN frame: ID=0x" << std::hex << frame_id << std::dec
              << ", Data=";
    for (size_t i = 0; i < data.size(); ++i)
    {
        std::cout << std::hex << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
    return true;
}

void CANInterface::registerFeedbackCallback(FeedbackCallback cb)
{
    callback_ = cb;
}

void CANInterface::receiveLoop()
{
    if (can_socket_ < 0)
    {
        std::cerr << "Error: Socket is not open. Call openInterface() first." << std::endl;
        return;
    }

    struct can_frame frame;
    while (true)
    {
        int nbytes = read(can_socket_, &frame, sizeof(frame));
        if (nbytes < 0)
        {
            std::cerr << "Error: Failed to read from CAN socket." << std::endl;
            break;
        }
        else if (nbytes < static_cast<int>(sizeof(frame)))
        {
            std::cerr << "Error: Incomplete CAN frame received." << std::endl;
            continue;
        }

        if (callback_)
        {
            uint8_t data[8];
            std::memcpy(data, frame.data, frame.can_dlc);
            callback_(frame.can_id, data);
        }
    }
}
