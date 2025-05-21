#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include "robot_interfaces/msg/imu.hpp"
#include "robot_interfaces/msg/base_cmd.hpp"
#include "robot_interfaces/srv/rotate_base.hpp"
#include "robot_interfaces/srv/push_ball.hpp"
#include "robot_interfaces/srv/request_mcu.hpp"
#include <thread>

#define FRAME_IDLE            {0x99, 0x02, 0x00, 0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_CLOSED_LOOP     {0x99, 0x02, 0x00, 0x9C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_HOMING          {0x99, 0x02, 0x00, 0x9D, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_RESET_IMU       {0x99, 0x01, 0x00, 0x9B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_RESET_ENCODER   {0x99, 0x01, 0x02, 0x9D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_CLEAR_ERRORS    {0x99, 0x02, 0x01, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_MANUAL_MODE     {0x99, 0x02, 0x00, 0xA5, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_SEMI_AUTO_MODE  {0x99, 0x02, 0x00, 0xA6, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_ROTATE_MODE     {0x99, 0x02, 0x00, 0xA7, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_EMERGENCY_STOP  {0x99, 0x02, 0x00, 0xAA, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_PUSH_BALL       {0x99, 0x02, 0x00, 0xA8, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_AUTO_IDLE       {0x99, 0x02, 0x11, 0xAC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_GO_BACK         {0x99, 0x02, 0x11, 0xAD, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_GO_STRAIGHT     {0x99, 0x02, 0x11, 0xAE, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_TURN_LEFT       {0x99, 0x02, 0x11, 0xAF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_TURN_RIGHT      {0x99, 0x02, 0x11, 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define FRAME_180_ROTATE      {0x99, 0x02, 0x11, 0xB1, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

/**
 * @brief UARTNode giao tiếp UART với vi điều khiển qua USB-TTL, nhận và gửi dữ liệu định dạng 12 byte.
 */
class UARTNode : public rclcpp::Node {
public:
    UARTNode(); ///< Hàm khởi tạo node ROS2

private:
    // ==== Cổng UART ====
    void configure_port(int fd, speed_t baudrate); ///< Cấu hình tốc độ truyền UART

    // ==== Mã hóa dữ liệu ====
    void float32_to_little_endian_8byte(float value, uint8_t out[8]); ///< Chuyển float32 thành 8 byte little endian

    // ==== Hàng đợi gửi UART ====
    void enqueue_uart_packet(uint8_t cmd, const uint8_t* data8);
    void Senqueue_uart_packet(uint8_t cmd, const uint8_t* data8); ///< Đóng gói frame UART và đưa vào hàng đợi
    void process_uart_queue(); ///< Gửi từng frame từ hàng đợi qua UART

    // ==== Gửi lệnh khởi tạo ====
    void send_initialization_commands(); ///< Gửi các gói cấu hình ban đầu đến MCU

    // ==== Giao tiếp ROS ====
    void handle_base_cmd(const robot_interfaces::msg::BaseCmd::SharedPtr msg); ///< Xử lý base_cmd (subscriber)
    void handle_rotate_service(const std::shared_ptr<robot_interfaces::srv::RotateBase::Request> request,
                               std::shared_ptr<robot_interfaces::srv::RotateBase::Response> response); ///< Gửi góc quay từ service
    void handle_push_ball_service(const std::shared_ptr<robot_interfaces::srv::PushBall::Request> request,
                                  std::shared_ptr<robot_interfaces::srv::PushBall::Response> response); ///< Gửi tín hiệu đẩy bóng khi được yêu cầu
    void handle_request_mcu_service(const std::shared_ptr<robot_interfaces::srv::RequestMcu::Request> request,
                                     std::shared_ptr<robot_interfaces::srv::RequestMcu::Response> response); ///< Xử lý lệnh điều khiển cơ sở từ service

    // ==== Nhận dữ liệu UART ====
    float convert_to_angle(uint8_t low, uint8_t high); ///< Giải mã góc từ 2 byte
    void uart_read_loop(); ///< Đọc UART, ghép frame và publish topic IMU

    // ==== Biến thành viên ====
    int uart_fd_;
    std::vector<uint8_t> buffer_;
    bool syncing_ = false;
    std::mutex uart_mutex_;
    std::mutex uart_queue_mutex_;
    std::queue<std::vector<uint8_t>> uart_queue_; // thang nay se luu gia tri cua tu topic base_cmd
    std::queue<std::vector<uint8_t>> Suart_queue_; // thang nay se luu gia tri cua 2 service kia va thang nay se duoc uu tien hon do no gui it
    std::queue<std::vector<uint8_t>> SSuart_queue_;  
    int mode_state; // Mode state cho service base_control

    // ==== ROS2 ====
    rclcpp::Publisher<robot_interfaces::msg::IMU>::SharedPtr pub_imu_;
    rclcpp::Service<robot_interfaces::srv::RotateBase>::SharedPtr service_rotate_;
    rclcpp::Service<robot_interfaces::srv::RequestMcu>::SharedPtr service_request_mcu_;
    rclcpp::Service<robot_interfaces::srv::PushBall>::SharedPtr service_push_ball_;
    rclcpp::Subscription<robot_interfaces::msg::BaseCmd>::SharedPtr sub_base_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr uart_tx_timer_;
    std::thread uart_read_thread_;

};
