#include <rclcpp/rclcpp.hpp>
#include <gamepad_interface/mapper.hpp>

using namespace std;

class GamepadNode : public rclcpp::Node
{
public:
  GamepadNode() : Node("gamepad_node"), driver_(declare_parameter<string>("device_path", DualSenseDriver::auto_detect())), mapper_(4.0f)
  {
    base_cmd_pub_ = create_publisher<robot_interfaces::msg::BaseCmd>("base_cmd", 10);
    timer_ = create_wall_timer(chrono::milliseconds(2), bind(&GamepadNode::loop, this));
  }

private:
  void loop()
  {
    GamepadState st;
    if (!driver_.read(st))
      return;

    MapperOutput mo = mapper_.update(st);

    if (mo.has_base_cmd)
      base_cmd_pub_->publish(mo.base_cmd);

    if (mo.has_request_mcu)
    {
      auto cli = get_request_mcu_client();
      if (cli->service_is_ready())
      {
        auto req = std::make_shared<robot_interfaces::srv::RequestMcu::Request>();
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
        auto req = std::make_shared<robot_interfaces::srv::RequestAction::Request>();
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
        auto req = std::make_shared<robot_interfaces::srv::RequestOdrive::Request>();
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
  rclcpp::Client<robot_interfaces::srv::RequestMcu>::SharedPtr
  get_request_mcu_client()
  {
    if (!req_mcu_cli_)
      req_mcu_cli_ = create_client<robot_interfaces::srv::RequestMcu>("request_mcu");
    return req_mcu_cli_;
  }

  rclcpp::Client<robot_interfaces::srv::RequestAction>::SharedPtr
  get_request_action_client()
  {
    if (!req_act_cli_)
      req_act_cli_ = create_client<robot_interfaces::srv::RequestAction>("request_action");
    return req_act_cli_;
  }

  rclcpp::Client<robot_interfaces::srv::RequestOdrive>::SharedPtr
  get_request_odrive_client()
  {
    if (!req_odrv_cli_)
      req_odrv_cli_ = create_client<robot_interfaces::srv::RequestOdrive>("request_odrive");
    return req_odrv_cli_;
  }

  // -------------- members ----------------
  DualSenseDriver driver_;
  RobotInputMapper mapper_;

  rclcpp::Publisher<robot_interfaces::msg::BaseCmd>::SharedPtr base_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<robot_interfaces::srv::RequestMcu>::SharedPtr req_mcu_cli_;
  rclcpp::Client<robot_interfaces::srv::RequestAction>::SharedPtr req_act_cli_;
  rclcpp::Client<robot_interfaces::srv::RequestOdrive>::SharedPtr req_odrv_cli_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadNode>());
  rclcpp::shutdown();
  return 0;
}
