#include "odrive_interface/odrive_motor.hpp"
#include <cstring>
#include <iostream>

std::vector<uint8_t> OdriveMotor::floatToBytes(float value)
{
    std::vector<uint8_t> bytes(sizeof(float));
    std::memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

uint32_t OdriveMotor::computeFrameId(uint8_t cmd_id)
{
    return (static_cast<uint32_t>(device_id_) << 5) | cmd_id;
}

OdriveMotor::OdriveMotor(uint8_t device_id, ControlMode mode, CANInterface *can_interface)
    : device_id_(device_id), mode_(mode), can_interface_(can_interface) {}

OdriveMotor::~OdriveMotor() {}

bool OdriveMotor::sendCommand(uint8_t cmd_id, const std::vector<uint8_t> &data)
{
    uint32_t frame_id = computeFrameId(cmd_id);
    if (!can_interface_->sendFrame(frame_id, data))
    {
        std::cerr << "Failed to send cmd_id=0x" << std::hex << static_cast<int>(cmd_id) << std::dec << std::endl;
        return false;
    }
    return true;
}

bool OdriveMotor::fullCalibration()
{
    return sendCommand(0x07, {0x03});
}

bool OdriveMotor::idle()
{
    return sendCommand(0x07, {0x01});
}

bool OdriveMotor::closeLoopControl()
{
    return sendCommand(0x07, {0x08});
}

bool OdriveMotor::clearError()
{
    return sendCommand(0x18, {0x00});
}

bool OdriveMotor::setHoming()
{
    return sendCommand(0x07, {0x0B});
}

bool OdriveMotor::setTarget(float value)
{
    uint8_t cmd_id = 0;
    switch (mode_)
    {
    case VELOCITY:
        cmd_id = 0x0D;
        break;
    case POSITION:
        cmd_id = 0x0C;
        break;
    case TORQUE:
        cmd_id = 0x0E;
        break;
    }
    return sendCommand(cmd_id, floatToBytes(value));
}

uint8_t OdriveMotor::getDeviceId() const
{
    return device_id_;
}

void OdriveMotor::setFeedback(float pos, float vel)
{
    position_ = pos;
    velocity_ = vel;
}

float OdriveMotor::getVelocity() const
{
    return velocity_;
}

float OdriveMotor::getPosition() const
{
    return position_;
}
