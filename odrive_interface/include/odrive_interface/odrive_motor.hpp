#ifndef ODRIVE_MOTOR_HPP_
#define ODRIVE_MOTOR_HPP_

#include <cstdint>
#include <vector>
#include "can_comm.hpp"

class OdriveMotor
{
public:
    enum ControlMode
    {
        VELOCITY,
        POSITION,
        TORQUE
    };

    static constexpr uint8_t FEEDBACK_CMD_ID = 0x09;

    OdriveMotor(uint8_t device_id, ControlMode mode, CANInterface *can_interface);
    ~OdriveMotor();

    bool fullCalibration();      // cmd 0x07, data=0x03
    bool idle();                 // cmd 0x07, data=0x01
    bool closeLoopControl();     // cmd 0x07, data=0x08
    bool clearError();           // cmd 0x18, data=0x00
    bool setHoming();            // cmd 0x07, data=0x0B
    bool setTarget(float value); // cmd based on mode

    uint8_t getDeviceId() const;
    void setFeedback(float pos, float vel);

    float getVelocity() const;
    float getPosition() const;

private:
    uint8_t device_id_;
    ControlMode mode_;
    CANInterface *can_interface_;

    float velocity_{0.0f};
    float position_{0.0f};

    uint32_t computeFrameId(uint8_t cmd_id);
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t> &data);
    std::vector<uint8_t> floatToBytes(float value);
};

#endif // ODRIVE_MOTOR_HPP_
