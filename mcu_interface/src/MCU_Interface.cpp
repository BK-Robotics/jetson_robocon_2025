#include "mcu_interface/MCU_Interface.hpp"

UARTNode::UARTNode() : Node("uart_node") {
    uart_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyUSB0");
        rclcpp::shutdown();
        return;
    }

    this->mode_state = 1;

    configure_port(uart_fd_, B115200);

    pub_imu_ = this->create_publisher<robot_interfaces::msg::IMU>("/imu", 10);

    service_rotate_ = this->create_service<robot_interfaces::srv::RotateBase>(
        "/rotate_base",
        std::bind(&UARTNode::handle_rotate_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    service_request_mcu_ = this->create_service<robot_interfaces::srv::RequestMcu>(
        "/request_mcu",
        std::bind(&UARTNode::handle_request_mcu_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    service_push_ball_ = this->create_service<robot_interfaces::srv::PushBall>(
        "/push_ball",
        std::bind(&UARTNode::handle_push_ball_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    sub_base_cmd_ = this->create_subscription<robot_interfaces::msg::BaseCmd>(
        "/base_cmd", 10,
        std::bind(&UARTNode::handle_base_cmd, this, std::placeholders::_1)
    );

    uart_read_thread_ = std::thread(&UARTNode::uart_read_loop, this);
    uart_read_thread_.detach();

    // setup a timer to process the UART queue every 2ms (because baudrate is 115200 so 12byte take 2ms is enough)
    uart_tx_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2), std::bind(&UARTNode::process_uart_queue, this)
    );

    send_initialization_commands();

    RCLCPP_INFO(this->get_logger(), "UART MCU node started.");
}

void UARTNode::configure_port(int fd, speed_t baudrate) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
}

void UARTNode::float32_to_little_endian_8byte(float value, uint8_t out[8]) {
    memset(out, 0, 8);
    uint8_t* float_ptr = reinterpret_cast<uint8_t*>(&value);
    for (int i = 0; i < 4; ++i) {
        out[i] = float_ptr[i];
    }
}

void UARTNode::enqueue_uart_packet(uint8_t cmd, const uint8_t* data8) {
    std::vector<uint8_t> frame = {0x99, 0x02, cmd, 0};
    for (int i = 0; i < 8; ++i) frame.push_back(data8[i]);
    uint8_t checksum = 0;
    for (int i = 0; i < 12; ++i) if (i != 3) checksum += frame[i];
    frame[3] = checksum % 256;
    std::lock_guard<std::mutex> lock(uart_queue_mutex_);
    uart_queue_.push(frame);
}

void UARTNode::Senqueue_uart_packet(uint8_t cmd, const uint8_t* data8) {
    std::vector<uint8_t> frame = {0x99, 0x02, cmd, 0};
    for (int i = 0; i < 8; ++i) frame.push_back(data8[i]);
    uint8_t checksum = 0;
    for (int i = 0; i < 12; ++i) if (i != 3) checksum += frame[i];
    frame[3] = checksum % 256;
    std::lock_guard<std::mutex> lock(uart_queue_mutex_);
    Suart_queue_.push(frame);
}

void UARTNode::process_uart_queue() {
    std::lock_guard<std::mutex> lock(uart_queue_mutex_);

    if(!SSuart_queue_.empty()) {
        std::vector<uint8_t> frame = SSuart_queue_.front();
        SSuart_queue_.pop();

        if (SSuart_queue_.size() >= 5) {
            RCLCPP_WARN(this->get_logger(), "SSUART queue is full, dropping packet.");
            while(!SSuart_queue_.empty()) {
                SSuart_queue_.pop();
            }
        }

        write(uart_fd_, frame.data(), frame.size());

        std::ostringstream oss;
        oss << "UART TX [";
        for (size_t i = 0; i < frame.size(); ++i) {
            oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(frame[i]);
            if (i != frame.size() - 1) oss << " ";
        }
        oss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }
    else if(!Suart_queue_.empty()) {
        std::vector<uint8_t> frame = Suart_queue_.front();
        Suart_queue_.pop();

        if (Suart_queue_.size() >= 5) {
            RCLCPP_WARN(this->get_logger(), "SUART queue is full, dropping packet.");
            while(!Suart_queue_.empty()) {
                Suart_queue_.pop();
            }
        }

        write(uart_fd_, frame.data(), frame.size());

        std::ostringstream oss;
        oss << "UART TX [";
        for (size_t i = 0; i < frame.size(); ++i) {
            oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(frame[i]);
            if (i != frame.size() - 1) oss << " ";
        }
        oss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }
    else{
        if (!uart_queue_.empty()) {
            std::vector<uint8_t> frame = uart_queue_.front();
            uart_queue_.pop();

            if (uart_queue_.size() >= 10) {
                RCLCPP_WARN(this->get_logger(), "UART queue is full, dropping packet.");
                while(!uart_queue_.empty()) {
                    uart_queue_.pop();
                }
            }

            write(uart_fd_, frame.data(), frame.size());

            std::ostringstream oss;
            oss << "UART TX [";
            for (size_t i = 0; i < frame.size(); ++i) {
                oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                << static_cast<int>(frame[i]);
                if (i != frame.size() - 1) oss << " ";
            }
            oss << "]";
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        }
    }
}

void UARTNode::send_initialization_commands() {
    std::vector<std::vector<uint8_t>> init_cmds = {
        {0x99, 0x01, 0x05, 0x71, 0x64, 0x00, 0x64, 0x00, 0x0A, 0x00, 0x00, 0x00},
        {0x99, 0x01, 0x00, 0x9B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x99, 0x01, 0x00, 0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x99, 0x01, 0x02, 0x9D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    };
    for (auto& frame : init_cmds){
        Suart_queue_.push(frame);
    }
}

// Cai data rot nay t dang thuc thi theo kieu xoay 4 huong trai, phai, stop_turn, back tozero nhe don.
void UARTNode::handle_base_cmd(const robot_interfaces::msg::BaseCmd::SharedPtr msg) {
    uint8_t data_vel[8];
    float32_to_little_endian_8byte(msg->velocity, data_vel);
    enqueue_uart_packet(0x0E, data_vel);

    uint8_t data_ang[8];
    float32_to_little_endian_8byte(msg->angle, data_ang);
    enqueue_uart_packet(0x0F, data_ang);

    uint8_t data_rot[8] = {msg->rotate, 0, 0, 0, 0, 0, 0, 0};
    enqueue_uart_packet(0x10, data_rot);
}

// cai rot nay la t thuc thi theo kieu assign nhe tai anh thinh gui theo kieu float.
void UARTNode::handle_rotate_service(const std::shared_ptr<robot_interfaces::srv::RotateBase::Request> request,
                                     std::shared_ptr<robot_interfaces::srv::RotateBase::Response> response) {
    SSuart_queue_.push(FRAME_ROTATE_MODE);
    float angle = request->angle;
    uint8_t data[8];
    float32_to_little_endian_8byte(angle, data);
    for(int i = 5; i > 0; --i) {
        data[i] = data[i-1];
    }
    data[0] = 0x04;

    Senqueue_uart_packet(0x10, data);
    response->success = true;
}

void UARTNode::handle_request_mcu_service(
    const std::shared_ptr<robot_interfaces::srv::RequestMcu::Request> request,
    std::shared_ptr<robot_interfaces::srv::RequestMcu::Response> response) {
    uint8_t cmd = request->action;
    std::vector<std::vector<uint8_t>> frames;

    switch (cmd) {
        case 0: frames.push_back(FRAME_IDLE); break;
        case 1: frames.push_back(FRAME_CLOSED_LOOP); break;
        case 2: frames.push_back(FRAME_HOMING); break;
        case 3: frames.push_back(FRAME_RESET_IMU); frames.push_back(FRAME_RESET_ENCODER); break;
        case 4: frames.push_back(FRAME_CLEAR_ERRORS); break;
        case 5: {
            if (this->mode_state == 0) frames.push_back(FRAME_MANUAL_MODE);
            else if (this->mode_state == 1) frames.push_back(FRAME_SEMI_AUTO_MODE);
            this->mode_state = (this->mode_state + 1) % 2;
            break;
        }
        case 6: frames.push_back(FRAME_AUTO_IDLE); break; // set up lai chu trinh 1
        case 7: frames.push_back(FRAME_GO_BACK); break; // set up lai chu trinh 2
        case 8: frames.push_back(FRAME_GO_STRAIGHT); break; // set up lai chu trinh 3
        case 9: frames.push_back(FRAME_TURN_LEFT); break; // set up lai chu trinh 4
        case 10: frames.push_back(FRAME_EMERGENCY_STOP); break;

        default:
            RCLCPP_WARN(this->get_logger(), "Unknown base_control cmd: %d", cmd);
            response->success = false;
            return;
    }

    {
        std::lock_guard<std::mutex> lock(uart_queue_mutex_);
        for (const auto& frame : frames) {
            SSuart_queue_.push(frame);
        }
    }

    response->success = true;
}

void UARTNode::handle_push_ball_service(const std::shared_ptr<robot_interfaces::srv::PushBall::Request> request,
                                        std::shared_ptr<robot_interfaces::srv::PushBall::Response> response) {
    if (!request->wait_for_completion) {
        response->success = false;
        return;
    }
    std::vector<uint8_t> frame = FRAME_PUSH_BALL;
    std::lock_guard<std::mutex> lock(uart_queue_mutex_);
    Suart_queue_.push(frame);
    response->success = true;

    this->mode_state = 0;
}

float UARTNode::convert_to_angle(uint8_t low, uint8_t high) {
    return static_cast<float>((int16_t)((high << 8) | low)) / 32768.0f * 180.0f;
}

void UARTNode::uart_read_loop() {
    while (rclcpp::ok()) {
        uint8_t byte;
        int n = read(uart_fd_, &byte, 1);
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        if (!syncing_) {
            if (byte == 0x99) {
                buffer_.clear();
                buffer_.push_back(byte);
                syncing_ = true;
            }
        } else {
            buffer_.push_back(byte);
            if (buffer_.size() == 3 && buffer_[2] != 0x01) {
                syncing_ = false;
                buffer_.clear();
                continue;
            }

            if (buffer_.size() == 12) {
                syncing_ = false;
                if (buffer_[2] == 0x01) {
                    uint8_t checksum = 0;
                    for (int i = 0; i < 12; ++i) {
                        if (i != 3) checksum += buffer_[i];
                    }
                    if (buffer_[3] != (checksum % 256)) {
                        std::stringstream ss;
                        ss << "Checksum error! Buffer = [ ";
                        for (uint8_t b : buffer_) {
                            ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
                        }
                        ss << "]";
                        RCLCPP_WARN(this->get_logger(), "%s", ss.str().c_str());
                        continue;
                    }

                    // std::stringstream ss;
                    // ss << "Buffer receive = [ ";
                    // for (uint8_t b : buffer_) {
                    //     ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
                    // }
                    // ss << "]";
                    // RCLCPP_WARN(this->get_logger(), "%s", ss.str().c_str());
                    
                    float z = convert_to_angle(buffer_[8], buffer_[9]);
                    robot_interfaces::msg::IMU msg;
                    msg.angle = z;
                    pub_imu_->publish(msg);
                }
            }
        }
    }
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UARTNode>());
    rclcpp::shutdown();
    return 0;
}
