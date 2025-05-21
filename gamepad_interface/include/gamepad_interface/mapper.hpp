#pragma once
#include <gamepad_interface/gamepad.hpp>
#include <robot_interfaces/msg/base_cmd.hpp>
#include <robot_interfaces/srv/request_mcu.hpp>
#include <robot_interfaces/srv/request_action.hpp>
#include <robot_interfaces/srv/request_odrive.hpp>
#include <optional>

using namespace std;

struct MapperOutput
{
    bool has_base_cmd{false};
    robot_interfaces::msg::BaseCmd base_cmd;

    bool has_request_mcu{false};
    uint8_t request_mcu{0};

    bool has_request_action{false};
    uint8_t request_action{0};

    bool has_request_odrive{false};
    uint8_t request_odrive{0};
};

class RobotInputMapper
{
public:
    explicit RobotInputMapper(float max_speed = 4.0f);

    /** Cập nhật theo GamepadState, trả về MapperOutput **/
    MapperOutput update(const GamepadState &s);

private:
    // —— helpers ————————————————————————————————
    float max_speed_;
    bool semi_auto_{false};   // PS toggles mode
    bool dpad_locked_{false}; // “first-press” handling
    int8_t last_dpad_x_{0};
    int8_t last_dpad_y_{0};
    std::array<uint8_t, 13> last_btn_{{0}};

    float prev_raw_angle_{0.0f};
    float cumulative_angle_{0.0f};
    bool first_angle_read_{true};
};
