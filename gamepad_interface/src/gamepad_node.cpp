#include <rclcpp/rclcpp.hpp>
#include <gamepad_interface/mapper.hpp>
#include <cmath>
#include <chrono>

using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;
using namespace robot_interfaces::msg;
using namespace robot_interfaces::srv;

class GamepadNode : public Node
{
public:
  GamepadNode()
      : Node("gamepad_interface"),
        driver_(declare_parameter<string>("device_path", DualSenseDriver::auto_detect())),
        mapper_(4.0f),
        last_send_time_(std::chrono::steady_clock::now())
  {
    timer_ = create_wall_timer(
        8ms,
        bind(&GamepadNode::loop, this));
    base_cmd_pub_ = this->create_publisher<BaseCmd>(
        "/base_cmd", 10);

    last_base_cmd_.velocity = 0.0f;
    last_base_cmd_.angle = 0.0f;
  }

private:
  void loop()
  {
    // Read gamepad state
    GamepadState st;
    if (!driver_.read(st))
      return;

    // Get output from mapper
    MapperOutput mo = mapper_.update(st);

    // Time passes from the last published
    auto now = chrono::steady_clock::now();
    constexpr auto MIN_INTERVAL = 8ms;
    if (now - last_send_time_ < MIN_INTERVAL)
    {
      return;
    }
    // constexpr float VEL_DEADBAND = 0.0104f;
    // constexpr float ANGLE_DEADBAND = 0.1604f;

    // bool base_cmd_changed = false;
    // if (mo.has_base_cmd)
    // {
    //   const auto &cmd = mo.base_cmd;
    //   if (fabs(cmd.velocity - last_base_cmd_.velocity) > VEL_DEADBAND &&
    //       fabs(cmd.angle - last_base_cmd_.angle) > ANGLE_DEADBAND)
    //   {
    //     base_cmd_changed = true;
    //   }
    // }

    // if (mo.has_base_cmd && base_cmd_changed)
    // {
    //   base_cmd_pub_->publish(mo.base_cmd);
    // }
    if (mo.has_base_cmd)
    {
      base_cmd_pub_->publish(mo.base_cmd);
    }
    if (mo.has_request_mcu)
    {
      auto cli = get_request_mcu_client();
      if (cli->service_is_ready())
      {
        auto req = std::make_shared<RequestMcu::Request>();
        req->action = mo.request_mcu;
        cli->async_send_request(req);
        RCLCPP_INFO(get_logger(), "[Service] Sending RequestMcu with action: %d", req->action);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "RequestMcu service is not ready.");
      }
    }
    if (mo.has_request_action)
    {
      auto cli = get_request_action_client();
      if (cli->service_is_ready())
      {
        auto req = std::make_shared<RequestAction::Request>();
        req->action = mo.request_action;
        cli->async_send_request(req);
        RCLCPP_INFO(get_logger(), "[Service] Sending RequestAction with action: %d", req->action);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "RequestAction service is not ready.");
      }
    }
    if (mo.has_request_odrive)
    {
      auto cli = get_request_odrive_client();
      if (cli->service_is_ready())
      {
        auto req = std::make_shared<RequestOdrive::Request>();
        req->action = mo.request_odrive;
        cli->async_send_request(req);
        RCLCPP_INFO(get_logger(), "[Service] Sending RequestOdrive with action: %d", req->action);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "RequestOdrive service is not ready.");
      }
    }
  }

  /* lazy-init clients */
  Client<RequestMcu>::SharedPtr
  get_request_mcu_client()
  {
    if (!req_mcu_cli_)
      req_mcu_cli_ = create_client<RequestMcu>("request_mcu");
    return req_mcu_cli_;
  }

  Client<RequestAction>::SharedPtr
  get_request_action_client()
  {
    if (!req_act_cli_)
      req_act_cli_ = create_client<RequestAction>("request_action");
    return req_act_cli_;
  }

  Client<RequestOdrive>::SharedPtr
  get_request_odrive_client()
  {
    if (!req_odrv_cli_)
      req_odrv_cli_ = create_client<RequestOdrive>("request_odrive");
    return req_odrv_cli_;
  }

  // -------------- members ----------------
  DualSenseDriver driver_;
  RobotInputMapper mapper_;

  Publisher<robot_interfaces::msg::BaseCmd>::SharedPtr base_cmd_pub_;
  TimerBase::SharedPtr timer_;
  Client<RequestMcu>::SharedPtr req_mcu_cli_;
  Client<RequestAction>::SharedPtr req_act_cli_;
  Client<RequestOdrive>::SharedPtr req_odrv_cli_;
  chrono::steady_clock::time_point last_send_time_;
  BaseCmd last_base_cmd_;
};
int main(int argc, char **argv)
{
  init(argc, argv);
  spin(std::make_shared<GamepadNode>());
  shutdown();
  return 0;
}