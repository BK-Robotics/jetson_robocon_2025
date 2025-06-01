#include <rclcpp/rclcpp.hpp>
// #include "robot_interfaces/srv/control.hpp"
#include "robot_interfaces/srv/push_ball.hpp"
#include "robot_interfaces/srv/request_odrive.hpp"
#include "odrive_interface/can_comm.hpp"
#include "odrive_interface/odrive_motor.hpp"
#include <thread>
#include <chrono>
#include <vector>
using namespace std;
using namespace chrono_literals;
// using ControlSrv = robot_interfaces::srv::Control;
using PushBall = robot_interfaces::srv::PushBall;
using OdriveSrv = robot_interfaces::srv::RequestOdrive;

static constexpr float BRACE_ON_POS = -39.0f;
static constexpr float BRACE_OFF_POS = 0.0f;
static const vector<uint8_t> SHOOTER_MOTOR_IDS = {0, 1, 2};
static const vector<uint8_t> DRIBBLE_MOTOR_IDS = {3, 4};
static const uint8_t BRACE_MOTOR_ID = 5;
static constexpr float DRIBBLE_FWD_SPEED = 15.0f;
static constexpr float DRIBBLE_REV_SPEED = 8.0f;
static constexpr float DRIBBLE_STOP_SPEED = 0.0f;
static constexpr float RELOAD_SPEED = 5.0f;
static constexpr float FIRE_MANUAL_SPEED = 50.0f;
static constexpr auto DRIBBLE_FWD_DUR = 150ms;
static constexpr auto DRIBBLE_REV_DUR = 500ms;
static constexpr auto DRIBBLE_STAB_DUR = 500ms;
static constexpr float RELEASE_SPEED = 4.0f;
static constexpr auto RELEASE_DUR = 800ms;

class OdriveInterfaceNode : public rclcpp::Node
{
public:
  OdriveInterfaceNode()
      : Node("odrive_interface"), brace_on_(false), is_reload_(false)
  {
    // 1. Khai báo kèm giá trị mặc định
    this->declare_parameter<std::string>("can_port", "canX");

    // 2. Đọc lại ngay sau khi khai báo
    can_port = this->get_parameter("can_port").as_string();

    // --- CAN và các OdriveMotor ---
    can_iface_ = std::make_unique<CANInterface>();
    if (!can_iface_->openInterface(can_port))
    {
      RCLCPP_FATAL(get_logger(), "Could not open %s", can_port.c_str());
      throw runtime_error("CAN init failed");
    }
    // khởi tạo 6 motor
    for (uint8_t id = 0; id < 6; ++id)
    {
      auto mode = (id < 5
                       ? OdriveMotor::ControlMode::VELOCITY
                       : OdriveMotor::ControlMode::POSITION);
      motors_.push_back(
          std::make_unique<OdriveMotor>(id, mode, can_iface_.get()));
    }

    // // --- Service Server /control ---
    // control_srv_ = create_service<ControlSrv>(
    //     "control",
    //     bind(&OdriveInterfaceNode::on_control, this,
    //          placeholders::_1, placeholders::_2));

    // --- Service Server /request_odrive ---
    odrive_srv_ = create_service<OdriveSrv>(
        "request_odrive",
        bind(&OdriveInterfaceNode::on_odrive_request, this,
             placeholders::_1, placeholders::_2));

    // --- Client cho /push_ball ---
    push_ball_client_ = create_client<PushBall>("push_ball");

    RCLCPP_INFO(get_logger(), "odrive_interface ready.");
  }

  // void on_control(const shared_ptr<ControlSrv::Request> req,
  //                 shared_ptr<ControlSrv::Response> res)
  // {
  //   switch (req->action)
  //   {
  //   case 1:
  //     action_push_ball(req->velocity);
  //     break;
  //   case 2:
  //     action_toggle_brace();
  //     break;
  //   case 3:
  //     action_dribble();
  //     break;
  //   case 4:
  //     action_auto();
  //     break;
  //   default:
  //     RCLCPP_WARN(get_logger(), "Unknown action %u", req->action);
  //     res->success = false;
  //     return;
  //   }
  //   res->success = true; // nếu tới được đây coi như OK
  // }

  void on_odrive_request(const shared_ptr<OdriveSrv::Request> req,
                         shared_ptr<OdriveSrv::Response> res)
  {
    switch (req->action)
    {
    case 0:
      idle();
      break;
    case 1:
      closed_loop_control();
      break;
    case 2:
      homing();
      break;
    case 3:
      reset_motors();
      break;
    case 4:
      clear_errors();
      break;
    case 5:
      reload();
      break;
    case 6:
      fire_manual();
      break;
    case 7:
      action_toggle_brace();
      break;
    case 8:
      action_dribble();
      break;
    case 9:
      action_auto();
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown action %u", req->action);
      res->success = false;
      return;
    }
    res->success = true; // nếu tới được đây coi như OK
  }

  // ────────────────────────────────────────────────────────────────
  // ⮞ Implementations
  // ────────────────────────────────────────────────────────────────
  void reset_motors()
  {
    for (int id = 0; id < 6; id++)
      motors_[id]->setTarget(0.0f);
    RCLCPP_INFO(get_logger(), "Reset done");
    brace_on_ = false;
    is_reload_ = false;
  }

  void idle()
  {
    for (int id = 0; id < 6; id++)
      motors_[id]->idle();
    RCLCPP_INFO(get_logger(), "Idle done");
  }

  void closed_loop_control()
  {
    for (int id = 0; id < 6; id++)
      motors_[id]->closeLoopControl();
    RCLCPP_INFO(get_logger(), "Closed loop done");
  }

  void clear_errors()
  {
    for (int id = 0; id < 6; id++)
      motors_[id]->clearError();
    RCLCPP_INFO(get_logger(), "Clear errors done");
  }

  void homing()
  {
    motors_[BRACE_MOTOR_ID]->setHoming();
  }
  // ---------------------------------------------------------------
  // void action_push_ball(uint8_t vel)
  // {
  //   // 1) bắn 0-1-2 ở chế độ velocity
  //   for (auto id : SHOOTER_MOTOR_IDS)
  //     motors_[id]->setTarget(static_cast<float>(vel));
  //   RCLCPP_INFO(get_logger(), "Shooter speed set to %.1f", vel);

  //   // 2) sau 0.5 s gọi /push_ball – không block
  //   thread([this]()
  //          {
  //     this_thread::sleep_for(1000ms);

  //     auto req  = std::make_shared<PushBall::Request>();
  //     req->wait_for_completion = true;

  //     if (!push_ball_client_->wait_for_service(1s)) 
  //     {
  //       RCLCPP_ERROR(get_logger(), "/push_ball service unavailable");
  //       return;
  //     }
  //     push_ball_client_->async_send_request(req);
  //     RCLCPP_INFO(get_logger(), "Called /push_ball (async)"); })
  //       .detach();
  // }

  // ---------------------------------------------------------------
  void action_toggle_brace()
  {
    float target = (brace_on_ ? BRACE_OFF_POS : BRACE_ON_POS);
    if (!brace_on_)
      motors_[BRACE_MOTOR_ID]->setTarget(target);
    else
      motors_[BRACE_MOTOR_ID]->setHoming();
    brace_on_ = !brace_on_;
    RCLCPP_INFO(get_logger(), "Brace %s (pos=%.1f)",
                brace_on_ ? "ON" : "OFF", target);
  }

  // ---------------------------------------------------------------
  void action_release()
  {
    thread([this]()
           {
    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->setTarget(RELEASE_SPEED * ((id % 2) ? 1.f : -1.f));

    this_thread::sleep_for(RELEASE_DUR);

    // Stop dribble motors
    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->setTarget(DRIBBLE_STOP_SPEED);

    RCLCPP_INFO(get_logger(), "Release sequence done"); })
        .detach();
  }

  // ---------------------------------------------------------------
  void action_dribble()
  {
    if (!brace_on_)
    {
      action_release();
      return;
    }
    // Chạy trong thread riêng để callback /control không block
    thread([this]()
           {
        // 1. Forward 200 ms
        for (auto id : DRIBBLE_MOTOR_IDS)
            motors_[id]->setTarget(DRIBBLE_FWD_SPEED * ((id % 2) ? 1.f : -1.f));
        this_thread::sleep_for(DRIBBLE_FWD_DUR);

        // 2. Reverse 2 s
        for (auto id : DRIBBLE_MOTOR_IDS)
            motors_[id]->setTarget(DRIBBLE_REV_SPEED * ((id % 2) ? -1.f : 1.f));
        this_thread::sleep_for(DRIBBLE_REV_DUR);

        // // 3. Cho 1 motor quay ngược hướng (motor đầu tiên)
        // float opposite_speed = - (2 * DRIBBLE_REV_SPEED * ((DRIBBLE_MOTOR_IDS[1] % 2) ? -1.f : 1.f));
        // motors_[DRIBBLE_MOTOR_IDS[1]]->setTarget(opposite_speed);

        // this_thread::sleep_for(DRIBBLE_STAB_DUR);

        // // 4. Cuối cùng, dừng hết
        // for (auto id : DRIBBLE_MOTOR_IDS)
        //     motors_[id]->setTarget(DRIBBLE_STOP_SPEED);

        for (auto id : DRIBBLE_MOTOR_IDS)
          motors_[id]->setTarget(DRIBBLE_STOP_SPEED);

        RCLCPP_INFO(get_logger(), "Dribble sequence done"); })
        .detach();
  }
  // ---------------------------------------------------------------
  void action_auto()
  {
    thread([this]()
           {
    /**************** 1. BRACE ON nếu chưa ****************/
    if (!brace_on_) 
    {
      motors_[BRACE_MOTOR_ID]->setTarget(BRACE_ON_POS);
      brace_on_ = true;                       // cập nhật cờ
      RCLCPP_INFO(get_logger(), "Auto: brace ON");
      this_thread::sleep_for(200ms);     // cho cơ cấu ổn định một chút
    }

    /**************** 2. DRIBBLE (nhồi 2 s) ****************/
    // forward 
    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->setTarget(DRIBBLE_FWD_SPEED * ((id % 2) ? 1.f : -1.f));
    this_thread::sleep_for(DRIBBLE_FWD_DUR);

    // reverse 
    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->setTarget(DRIBBLE_REV_SPEED * ((id % 2) ? -1.f : 1.f));
    this_thread::sleep_for(DRIBBLE_REV_DUR);


    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->setTarget(DRIBBLE_STOP_SPEED);
    
    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->idle();
    this_thread::sleep_for(DRIBBLE_STAB_DUR);

    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->clearError();
    
    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->closeLoopControl();

    /**************** 3. BRACE OFF ****************/
    motors_[BRACE_MOTOR_ID]->setTarget(BRACE_OFF_POS);
    brace_on_ = false;
    RCLCPP_INFO(get_logger(), "Auto: brace OFF");

    /**************** 4. Đợi feedback vị trí = 0 ****************/
    constexpr float POS_EPS = 0.1f;                  // dung sai
    const auto      TIMEOUT = 5s;                    // tránh kẹt
    auto            start   = chrono::steady_clock::now();

    while (true) {
      float pos = motors_[BRACE_MOTOR_ID]->getPosition();  
      if (abs(pos - BRACE_OFF_POS) < POS_EPS) break;
      if (chrono::steady_clock::now() - start > TIMEOUT) 
      {
        RCLCPP_ERROR(get_logger(), "Auto: brace never reached 0 !");
        return;   // hủy quy trình
      }
      this_thread::sleep_for(50ms);
    }

    /**************** 5. RELEASE (speed = 4 trong 0,4 s) ****************/
    RCLCPP_INFO(get_logger(), "Auto: release");
    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->setTarget(RELEASE_SPEED * ((id % 2) ? 1.f : -1.f));
    this_thread::sleep_for(RELEASE_DUR);
    for (auto id : DRIBBLE_MOTOR_IDS)
      motors_[id]->setTarget(DRIBBLE_STOP_SPEED);

    RCLCPP_INFO(get_logger(), "Auto: sequence DONE"); })
        .detach();
  }

  void reload()
  {
    if (!is_reload_)
    {
      for (auto id : DRIBBLE_MOTOR_IDS)
        motors_[id]->setTarget(RELOAD_SPEED * ((id % 2) ? -1.f : 1.f));
      RCLCPP_INFO(get_logger(), "Reloading...");
      is_reload_ = true;
    }
    else
    {
      for (auto id : DRIBBLE_MOTOR_IDS)
        motors_[id]->setTarget(DRIBBLE_STOP_SPEED);
      RCLCPP_INFO(get_logger(), "Reload stopped");
      is_reload_ = false;
    }
  }

  void fire_manual()
  {
    // 1) bắn 0-1-2 ở chế độ velocity
    for (auto id : SHOOTER_MOTOR_IDS)
      motors_[id]->setTarget(FIRE_MANUAL_SPEED);
    RCLCPP_INFO(get_logger(), "Shooter speed set to %.1f", FIRE_MANUAL_SPEED);

    // 2) sau 0.5 s gọi /push_ball – không block
    thread([this]()
           {
      this_thread::sleep_for(777ms);

      auto req  = std::make_shared<PushBall::Request>();
      req->wait_for_completion = true;

      if (!push_ball_client_->wait_for_service(1s)) 
      {
        RCLCPP_ERROR(get_logger(), "/push_ball service unavailable");
        return;
      }
      push_ball_client_->async_send_request(req);
      RCLCPP_INFO(get_logger(), "Called /push_ball (async)"); })
        .detach();
  }
  // ────────────────────────────────────────────────────────────────
  // Private data
  // ────────────────────────────────────────────────────────────────
private:
  unique_ptr<CANInterface> can_iface_;
  vector<unique_ptr<OdriveMotor>> motors_;
  // rclcpp::Service<ControlSrv>::SharedPtr control_srv_;
  rclcpp::Service<OdriveSrv>::SharedPtr odrive_srv_;
  rclcpp::Client<PushBall>::SharedPtr push_ball_client_;
  bool brace_on_;
  bool is_reload_;
  string can_port;
};

// ────────────────────────────────────────────────────────────────
// main()
// ────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdriveInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}