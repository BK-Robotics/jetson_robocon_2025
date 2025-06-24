#include <rclcpp/rclcpp.hpp>
// #include "robot_interfaces/srv/control.hpp"
#include "robot_interfaces/srv/push_ball.hpp"
#include "robot_interfaces/srv/request_odrive.hpp"
#include "odrive_interface/can_comm.hpp"
#include "odrive_interface/odrive_motor.hpp"
#include <thread>
#include <chrono>
#include <vector>
#include <functional>

using namespace std;
using namespace chrono_literals;
// using ControlSrv = robot_interfaces::srv::Control;
using PushBall = robot_interfaces::srv::PushBall;
using OdriveSrv = robot_interfaces::srv::RequestOdrive;

static constexpr float BRACE_ON_POS = -38.0f;
static constexpr float BRACE_OFF_POS = 0.0f;
static const vector<uint8_t> SHOOTER_MOTOR_IDS = {0, 1, 2};
static const vector<uint8_t> DRIBBLE_MOTOR_IDS = {3, 4};
static const uint8_t BRACE_MOTOR_ID = 5;
static constexpr float DRIBBLE_FWD_SPEED = 25.0f;
static constexpr float DRIBBLE_REV_SPEED = 8.0f;
static constexpr float DRIBBLE_STOP_SPEED = 0.0f;
static constexpr float RELOAD_SPEED = 5.0f;
static constexpr float FIRE_MANUAL_SPEED = 50.0f;
static constexpr float PASS_SPEED = 20.0f;
static constexpr auto DRIBBLE_FWD_DUR = 150ms;
static constexpr auto DRIBBLE_REV_DUR = 600ms;
static constexpr auto DRIBBLE_STAB_DUR = 500ms;
static constexpr float RELEASE_SPEED = 4.0f;
static constexpr auto RELEASE_DUR = 800ms;

class OdriveInterfaceNode : public rclcpp::Node
{
public:
  OdriveInterfaceNode()
      : Node("odrive_interface"), brace_on_(false), is_reload_(false), fired_{false}, passed_{false}
  {
    this->declare_parameter<string>("can_port", "canX");
    can_port = this->get_parameter("can_port").as_string();

    this->declare_parameter<float>("shoot_speed", 20.0f);
    shoot_speed_ = this->get_parameter("shoot_speed").as_double();

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
    case 10:
      pass();
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown action %u", req->action);
      res->success = false;
      return;
    }
    res->success = true;
  }

  // ────────────────────────────────────────────────────────────────
  // ⮞ Implementations
  // ────────────────────────────────────────────────────────────────

  void reset_motors()
  {
    sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                  { motors_[id]->setTarget(0.0f); });
    RCLCPP_INFO(get_logger(), "Reset sequence started");
    brace_on_ = false;
    is_reload_ = false;
  }

  void idle()
  {
    vector<uint8_t> all_ids = {0, 1, 2, 3, 4, 5};
    sendThrottled(all_ids, [this](uint8_t id)
                  { motors_[id]->idle(); });
    RCLCPP_INFO(get_logger(), "Idle sequence started");
  }

  void closed_loop_control()
  {
    sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                  { motors_[id]->closeLoopControl(); });
    RCLCPP_INFO(get_logger(), "Closed-loop sequence started");
  }

  void clear_errors()
  {
    vector<uint8_t> all_ids = {0, 1, 2, 3, 4, 5};
    sendThrottled(all_ids, [this](uint8_t id)
                  { motors_[id]->clearError(); });
    RCLCPP_INFO(get_logger(), "Clear errors sequence started");
  }

  void homing()
  {
    // motors_[BRACE_MOTOR_ID]->setHoming();
    RCLCPP_INFO(get_logger(), "Homing done");
  }
  /*
    void action_dribble()
    {
      if (!brace_on_)
      {
        action_release();
        return;
      }
      thread([this]()
             {
               // 1. Forward (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->setTarget(DRIBBLE_FWD_SPEED * ((id % 2) ? 1.f : -1.f));
               });
               this_thread::sleep_for(DRIBBLE_FWD_DUR + chrono::milliseconds(5 * DRIBBLE_MOTOR_IDS.size()));

               // 2. Reverse (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->setTarget(DRIBBLE_REV_SPEED * ((id % 2) ? -1.f : 1.f));
               });
               this_thread::sleep_for(DRIBBLE_REV_DUR + chrono::milliseconds(5 * DRIBBLE_MOTOR_IDS.size()));

               // 3. Stop (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->setTarget(DRIBBLE_STOP_SPEED);
               });

               RCLCPP_INFO(get_logger(), "Dribble sequence done (throttled)"); })
          .detach();
    }

    void action_auto()
    {
      thread([this]()
             {
               // 1. BRACE ON nếu chưa
               if (!brace_on_)
               {
                 motors_[BRACE_MOTOR_ID]->setTarget(BRACE_ON_POS);
                 brace_on_ = true;
                 RCLCPP_INFO(get_logger(), "Auto: brace ON");
                 this_thread::sleep_for(200ms);
               }

               // 2. DRIBBLE: FWD (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->setTarget(DRIBBLE_FWD_SPEED * ((id % 2) ? 1.f : -1.f));
               });
               this_thread::sleep_for(DRIBBLE_FWD_DUR + chrono::milliseconds(5 * DRIBBLE_MOTOR_IDS.size()));

               // DRIBBLE: Reverse (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->setTarget(DRIBBLE_REV_SPEED * ((id % 2) ? -1.f : 1.f));
               });
               this_thread::sleep_for(DRIBBLE_REV_DUR + chrono::milliseconds(5 * DRIBBLE_MOTOR_IDS.size()));

               // Idle & dừng (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->setTarget(DRIBBLE_STOP_SPEED);
               });
               this_thread::sleep_for(DRIBBLE_STAB_DUR + chrono::milliseconds(5 * DRIBBLE_MOTOR_IDS.size()));

               // clearError (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->clearError();
               });
               this_thread::sleep_for(chrono::milliseconds(5 * DRIBBLE_MOTOR_IDS.size()));

               // closeLoopControl (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->closeLoopControl();
               });
               this_thread::sleep_for(chrono::milliseconds(5 * DRIBBLE_MOTOR_IDS.size()));

               // 3. BRACE OFF
               motors_[BRACE_MOTOR_ID]->setTarget(BRACE_OFF_POS);
               brace_on_ = false;
               RCLCPP_INFO(get_logger(), "Auto: brace OFF");

               // 4. Đợi feedback vị trí = 0
               constexpr float POS_EPS = 0.1f;
               const auto      TIMEOUT = 5s;
               auto            start   = chrono::steady_clock::now();
               while (true) {
                 float pos = motors_[BRACE_MOTOR_ID]->getPosition();
                 if (abs(pos - BRACE_OFF_POS) < POS_EPS) break;
                 if (chrono::steady_clock::now() - start > TIMEOUT) {
                   RCLCPP_ERROR(get_logger(), "Auto: brace never reached 0 !");
                   return;
                 }
                 this_thread::sleep_for(50ms);
               }

               // 5. RELEASE (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->setTarget(RELEASE_SPEED * ((id % 2) ? 1.f : -1.f));
               });
               this_thread::sleep_for(RELEASE_DUR + chrono::milliseconds(5 * DRIBBLE_MOTOR_IDS.size()));

               // STOP release (throttled)
               sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id) {
                 motors_[id]->setTarget(DRIBBLE_STOP_SPEED);
               });

               RCLCPP_INFO(get_logger(), "Auto: sequence DONE (throttled)"); })
          .detach();
    }

    void reload()
    {
      if (!is_reload_)
      {
        sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->setTarget(RELOAD_SPEED * ((id % 2) ? -1.f : 1.f)); });
        RCLCPP_INFO(get_logger(), "Reloading sequence started (throttled)");
        is_reload_ = true;
      }
      else
      {
        sendThrottled(DRIBBLE_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->setTarget(DRIBBLE_STOP_SPEED); });
        RCLCPP_INFO(get_logger(), "Reload stopped (throttled)");
        is_reload_ = false;
      }
    }

    void fire_manual()
    {
      if (!fired_)
      {
        sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->setTarget(FIRE_MANUAL_SPEED); });
        RCLCPP_INFO(get_logger(), "Shooter speed sequence started: %.1f (throttled)", FIRE_MANUAL_SPEED);
  =
        thread([this]()
               {
                 this_thread::sleep_for(999ms);

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
      else
      {
        sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->setTarget(0); });
        RCLCPP_INFO(get_logger(), "Shooter stop sequence started (throttled)");
      }
      fired_ = !fired_;
    }
  */

  void pass()
  {
    if (!passed_)
    {

      thread([this]()
             {   
        sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->clearError(); });
        this_thread::sleep_for(100ms);

        sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->closeLoopControl(); });
        this_thread::sleep_for(5ms);

        sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->setTarget(shoot_speed_); });
        RCLCPP_INFO(get_logger(), "Passing sequence started: %.1f (throttled)", shoot_speed_);
        this_thread::sleep_for(333ms);

        auto req = std::make_shared<PushBall::Request>();
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
    else
    {
      thread([this]()
             {
        sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->setTarget(0); });
        this_thread::sleep_for(1000ms);

        sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                      { motors_[id]->clearError(); });
        this_thread::sleep_for(5ms);
        
        sendThrottled(SHOOTER_MOTOR_IDS, [this](uint8_t id)
                      { motors_[0]->findIndex();
                        motors_[1]->findIndex(); });
        motors_[2]->sendCommand(0x07, {0x07});
        
        RCLCPP_INFO(get_logger(), "Shooter stop sequence started"); })
          .detach();
    }
    passed_ = !passed_;
  }

private:
  unique_ptr<CANInterface> can_iface_;
  vector<unique_ptr<OdriveMotor>> motors_;
  // rclcpp::Service<ControlSrv>::SharedPtr control_srv_;
  rclcpp::Service<OdriveSrv>::SharedPtr odrive_srv_;
  rclcpp::Client<PushBall>::SharedPtr push_ball_client_;
  bool brace_on_;
  bool is_reload_;
  bool fired_;
  bool passed_;
  string can_port;
  float shoot_speed_;

  /**
   * Gửi lần lượt các lệnh lên motor với interval 5 ms giữa mỗi lần.
   */
  void sendThrottled(
      const vector<uint8_t> &ids,
      function<void(uint8_t)> action)
  {
    auto idx = std::make_shared<size_t>(0);

    auto timer_handle = std::make_shared<std::shared_ptr<rclcpp::TimerBase>>();

    function<void()> cb = [this, ids, action, idx, timer_handle]() mutable
    {
      if (*idx < ids.size())
      {
        action(ids[*idx]);
        (*idx)++;
      }
      else
      {
        if (timer_handle->get() != nullptr)
        {
          (*timer_handle)->cancel();
        }
      }
    };

    auto real_timer = this->create_wall_timer(
        chrono::milliseconds(5),
        cb);
    *timer_handle = real_timer;
  }
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