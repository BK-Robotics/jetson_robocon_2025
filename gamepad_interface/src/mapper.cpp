#include "gamepad_interface/mapper.hpp"
#include <cmath>
#include <limits>

using namespace std;
using namespace robot_interfaces::msg;
using namespace robot_interfaces::srv;

RobotInputMapper::RobotInputMapper(float max_speed)
    : max_speed_(max_speed) {}

MapperOutput RobotInputMapper::update(const GamepadState &s)
{
    MapperOutput out;

    /*---------------- GENERAL + MODE toggle ----------------*/
    auto edge = [&](int idx) -> bool
    {
        bool rising = (s.buttons[idx] == 1 && last_btn_[idx] == 0);
        last_btn_[idx] = s.buttons[idx];
        return rising;
    };

    if (edge(10))
    { // PS = BTN_MODE
        if (rotate_only_)
        { // đang ở rotate-only ⇒ thoát ra manual
            rotate_only_ = false;
            semi_auto_ = false;
        }
        else
        {
            semi_auto_ ^= 1; // toggle manual ↔ semi-auto
        }
        out.request_mcu = 5;
        out.has_request_mcu = true;
    }
    if (edge(0))
    { // Cross: Pass
        // rotate_only_ = true;
        // semi_auto_ = false;
        out.request_odrive = 10;
        out.has_request_odrive = true;
    }
    if (edge(2))
    { // Triangle: Brace
        // out.request_odrive = 7;
        // out.has_request_odrive = true;
    }
    if (edge(3))
    { // Square: Dribble
        // out.request_odrive = 8;
        // out.has_request_odrive = true;
    }
    if (edge(1))
    { // Circle: Auto
        // out.request_odrive = 9;
        // out.has_request_odrive = true;
    }
    if (edge(12))
    { // L3: Reload
        // out.request_odrive = 5;
        // out.has_request_odrive = true;
    }

    if (edge(4))
    { // L1: Idle
        out.request_mcu = 0;
        out.has_request_mcu = true;
        out.request_odrive = 0;
        out.has_request_odrive = true;
    }
    if (edge(5))
    { // R1: Closed Loop
        out.request_mcu = 1;
        out.has_request_mcu = true;
        out.request_odrive = 1;
        out.has_request_odrive = true;
    }
    if (edge(11))
    { // R3: Homing
        out.request_mcu = 2;
        out.has_request_mcu = true;
        out.request_odrive = 2;
        out.has_request_odrive = true;
    }
    if (edge(8))
    { // CREATE: Reset
        out.request_mcu = 3;
        out.has_request_mcu = true;
        out.request_odrive = 3;
        out.has_request_odrive = true;
    }
    if (edge(9))
    { // OPTIONS: ClearError
        out.request_mcu = 4;
        out.has_request_mcu = true;
        out.request_odrive = 4;
        out.has_request_odrive = true;
    }

    if (rotate_only_)
    {
        if (edge(13))
        { // Touch-pad: Fire (manual)
            out.request_odrive = 6;
            out.has_request_odrive = true;
        }
        BaseCmd cmd;
        /* rotate : D-Pad */
        int8_t dx = static_cast<int8_t>(s.axes[6]);
        int8_t dy = static_cast<int8_t>(s.axes[7]);

        if (dy == -1 && !dpad_locked_)
        {
            cmd.rotate = 0x00;
            dpad_locked_ = true;
        }
        else if (dx != 0)
        { // Left/Right
            cmd.rotate = (dx == -1 ? 0x01 : 0x02);
            dpad_locked_ = true;
        }
        else if (dx == 0 && dy == 0)
        {
            cmd.rotate = 0x03;    // Neutral/dừng
            dpad_locked_ = false; // unlock
        }
        out.base_cmd = cmd;
        out.has_base_cmd = true;
    }
    /*---------------- MANUAL vs SEMI-AUTO ----------------*/
    else if (!semi_auto_)
    { /************  MANUAL  ************/
        if (edge(13))
        { // Touch-pad: Fire (manual)
            out.request_odrive = 6;
            out.has_request_odrive = true;
        }

        BaseCmd cmd;

        /* velocity : L2 (-) + R2 (+) */
        constexpr float SAFEZONE = 0.1f;

        float t_neg = s.axes[5]; // L2
        float t_pos = s.axes[4]; // R2

        if (t_neg < SAFEZONE)
            t_neg = 0.0f;
        if (t_pos < SAFEZONE)
            t_pos = 0.0f;

        float raw = t_pos - t_neg;
        if (fabs(raw) < SAFEZONE)
        {
            raw = 0.0f;
        }
        cmd.velocity = raw * max_speed_;

        /* angle : L-Stick */
        float lx = s.axes[2], ly = s.axes[3];
        if (hypot(lx, ly) > 0.2f)
        {
            float raw = -atan2(-lx, -ly) * 180 / M_PI;

            if (first_angle_read_)
            {
                cumulative_angle_ = raw;
                prev_raw_angle_ = raw;
                first_angle_read_ = false;
            }
            else
            {
                float delta = raw - prev_raw_angle_;
                if (delta > 180.0f)
                    delta -= 360.0f;
                if (delta < -180.0f)
                    delta += 360.0f;
                cumulative_angle_ += delta;
                prev_raw_angle_ = raw;
            }
            cmd.angle = cumulative_angle_;
        }
        else
        {
            first_angle_read_ = true;
            cmd.angle = 0.0f;
        }

        /* rotate : D-Pad */
        int8_t dx = static_cast<int8_t>(s.axes[6]);
        int8_t dy = static_cast<int8_t>(s.axes[7]);

        if (dy == -1 && !dpad_locked_)
        {
            cmd.rotate = 0x00;
            dpad_locked_ = true;
        }
        else if (dx != 0)
        { // Left/Right
            cmd.rotate = (dx == -1 ? 0x01 : 0x02);
            dpad_locked_ = true;
        }
        else if (dx == 0 && dy == 0)
        {
            cmd.rotate = 0x03;    // Neutral/dừng
            dpad_locked_ = false; // unlock
        }

        out.base_cmd = cmd;
        out.has_base_cmd = true;
    }
    else
    { /************  SEMI-AUTO  ************/
        int8_t dx = static_cast<int8_t>(s.axes[6]);
        int8_t dy = static_cast<int8_t>(s.axes[7]);

        if (dx == -1 && !dpad_locked_)
        {
            out.request_mcu = 6;
            out.has_request_mcu = true;
            dpad_locked_ = true;
        }
        else if (dx == 1 && !dpad_locked_)
        {
            out.request_mcu = 7;
            out.has_request_mcu = true;
            dpad_locked_ = true;
        }
        else if (dy == -1 && !dpad_locked_)
        {
            out.request_mcu = 8;
            out.has_request_mcu = true;
            dpad_locked_ = true;
        }
        else if (dy == 1 && !dpad_locked_)
        {
            out.request_mcu = 9;
            out.has_request_mcu = true;
            dpad_locked_ = true;
        }
        else if (dx == 0 && dy == 0)
        {
            dpad_locked_ = false;
        }

        if (edge(13))
        { // Touch-Pad click
            out.request_mcu = 10;
            out.has_request_mcu = true;
            out.request_odrive = 0;
            out.has_request_odrive = true;
        }
    }

    return out;
}
