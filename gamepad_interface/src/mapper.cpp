#include "gamepad_interface/mapper.hpp"
#include <cmath>
#include <limits>

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
        semi_auto_ = !semi_auto_;
        out.request_mcu = 5;
        out.has_request_mcu = true;
        out.request_action = 5 + static_cast<int>(semi_auto_);
        out.has_request_action = true;
    }

    if (edge(0))
    { // Cross: Fire
        out.request_action = 1; 
        out.has_request_action = true;
    }
    if (edge(2))
    { // Triangle: Brace
        out.request_action = 2; 
        out.has_request_action = true;
    }
    if (edge(3))
    { // Square: Dribble
        out.request_action = 3; 
        out.has_request_action = true;
    }
    if (edge(1))
    { // Circle: Auto
        out.request_action = 4; 
        out.has_request_action = true;
    }

    if (edge(4))
    { // L1: Idle
        out.request_mcu = 0;
        out.has_request_mcu = true;
        out.request_odrive = 0;
        out.has_request_odrive = true;
        out.request_action = 7;
        out.has_request_action = true;
    }
    if (edge(5))
    { // R1: Closed Loop
        out.request_mcu = 1;
        out.has_request_mcu = true;
        out.request_odrive = 1;
        out.has_request_odrive = true;
    }
    if (edge(12))
    { // R3: Homing
        out.request_mcu = 2; 
        out.has_request_mcu = true;
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

    /*---------------- MANUAL vs SEMI-AUTO ----------------*/
    if (!semi_auto_)
    { /************  MANUAL  ************/
        robot_interfaces::msg::BaseCmd cmd;

        /* velocity : L2 (-) + R2 (+) */
        float v_neg = -s.axes[4] * max_speed_; // L2
        float v_pos = s.axes[5] * max_speed_;  // R2
        cmd.velocity = (fabs(v_pos) > fabs(v_neg)) ? v_pos : v_neg;

        /* angle : L-Stick */
        float lx = s.axes[0], ly = s.axes[1];
        if (hypot(lx, ly) > 0.1f)
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
            first_angle_read_= true;
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

        if (edge(12))
        { // Touch-Pad click
            out.request_mcu = 10;
            out.has_request_mcu = true;
            out.request_odrive = 0;
            out.has_request_odrive = true;
            out.request_action = 7;
            out.has_request_action = true;
        }
    }

    return out;
}
