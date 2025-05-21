#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP
#include <array>
#include <string>
#include <cstdint>

struct GamepadState
{
    std::array<float, 8> axes; 
    std::array<uint8_t, 13> buttons; 
};

class DualSenseDriver
{
public:
    explicit DualSenseDriver(const std::string &dev);
    bool read(GamepadState &out);
    static std::string auto_detect();
private:
    int fd_{-1};
    GamepadState state_{};
};

#endif
