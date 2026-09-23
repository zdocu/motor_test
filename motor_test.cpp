#include <cstddef>
#include <memory>
#include <span>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <rmcs_executor/component.hpp>
#include <librmcs/board/c_board.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/can_packet.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"

namespace rmcs_core::hardware {

class MotorTest
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    MotorTest()
        : Node{"motor_test",
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , motor_(*this, *command_, "/motor")
        , dr16_{}
        , velocity_filter_{10.0, 100.0} {

        // 配置GM6020电机：类型GM6020，ID=1，减速比1:1，多圈角度
        motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                .enable_multi_turn_angle());

        // 初始化板卡
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, "");
        board_->start_transmit();

        // 初始化遥控器
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        // 初始化PID（速度环）
        speed_pid_.kp = 0.005;
        speed_pid_.ki = 0.0;
        speed_pid_.kd = 0.0;
        speed_pid_.output_min = -0.05;
        speed_pid_.output_max = 0.05;

        // 初始化角度环PID
        angle_pid_.kp = 2.0;
        angle_pid_.ki = 0.01;
        angle_pid_.kd = 0.1;
        angle_pid_.output_min = -20.0;   // 最大速度±3 rad/s
        angle_pid_.output_max = 20.0;


        RCLCPP_INFO(logger_, "MotorTest initialized!");
    }

    MotorTest(const MotorTest&) = delete;
    MotorTest& operator=(const MotorTest&) = delete;
    MotorTest(MotorTest&&) = delete;
    MotorTest& operator=(MotorTest&&) = delete;

    ~MotorTest() override {
        // 退出的时候把扭矩设为0，防止猛震
        if (board_) {
            auto builder = board_->start_transmit();
            builder.can_transmit(
                Spec::kCans.kCan1,
                {
                    .can_id = 0x200,
                    .can_data = device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::Quarter{0},
                        device::CanPacket8::PaddingQuarter{},
                    }.as_bytes(),
                });
        }
    }

    
    void update() override {
        // 更新电机状态
        motor_.update_status();

        // 更新遥控器
        dr16_.update_status();
        remote_control_->update();

        // 读所有摇杆通道
        auto left_x = dr16_.joystick_left().x();
        auto left_y = dr16_.joystick_left().y();
        auto right_x = dr16_.joystick_right().x();
        auto right_y = dr16_.joystick_right().y();
        double stick_y = left_x;  // 这个遥控器推上下的时候left_x变，所以用left_x
        // 死区：小于0.1当成0
        if (std::abs(stick_y) < 0.1) stick_y = 0.0;

        static int debug_count = 0;
        if (debug_count++ % 1000 == 0) {
            RCLCPP_INFO(logger_, "DR16 valid: %d", dr16_.valid());
            RCLCPP_INFO(logger_, "L: (%.2f, %.2f) R: (%.2f, %.2f)", left_x, left_y, right_x, right_y);
        }



        // 低通滤波当前速度和角度
        double current_speed = velocity_filter_.update(motor_.velocity());

        // 根据右拨杆开关切换模式
        auto switch_right = dr16_.switch_right();
        auto switch_left = dr16_.switch_left();

        static int switch_debug_count = 0;
        if (switch_debug_count++ % 1000 == 0) {
            RCLCPP_INFO(logger_, "switch_left = %d, switch_right = %d", 
                       static_cast<int>(switch_left), static_cast<int>(switch_right));
        }

        if (switch_right == rmcs_msgs::Switch::UP) {
            // 速度模式
            double target_speed = stick_y * 10.0;
            double error = target_speed - current_speed;
            control_torque_ = speed_pid_.update(error);

            static int count = 0;
            if (count++ % 100 == 0) {
                RCLCPP_INFO(logger_,
                    "[速度模式] target=%.2f, current=%.2f, torque=%.2f",
                    target_speed, current_speed, control_torque_);
            }
        } 
        else if (switch_right == rmcs_msgs::Switch::DOWN) {
            // 角度模式
            static double target_angle = 0.0;
            target_angle += stick_y * 0.02;  

            double angle_error = target_angle - motor_.angle();
            double target_speed = angle_pid_.update(angle_error);

            double speed_error = target_speed - current_speed;
            control_torque_ = speed_pid_.update(speed_error);

            static int count = 0;
            if (count++ % 100 == 0) {
                RCLCPP_INFO(logger_,
                    "[角度模式] target=%.2f rad, current=%.2f rad, torque=%.2f",
                    target_angle, motor_.angle(), control_torque_);
            }
        }
        else {
            // 中间：停止
            control_torque_ = 0.0;
        }

    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        static int debug_can_count = 0;
        if (debug_can_count++ % 100 == 0) {
            std::string hex_str;
            for (size_t i = 0; i < data.can_data.size(); ++i) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02X ", static_cast<uint8_t>(data.can_data[i]));
                hex_str += buf;
            }
            RCLCPP_INFO(logger_, "CAN received, id=0x%X, data: %s", data.can_id, hex_str.c_str());
        }
        motor_.match_then_store_status(data.can_id, data.can_data);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus) {
            static int debug_uart_count = 0;
            if (debug_uart_count++ % 100 == 0) {
                RCLCPP_INFO(logger_, "DBUS data size: %zu", data.uart_data.size());
                std::string hex_str;
                for (size_t i = 0; i < data.uart_data.size(); ++i) {
                    char buf[4];
                    snprintf(buf, sizeof(buf), "%02X ", static_cast<uint8_t>(data.uart_data[i]));
                    hex_str += buf;
                }
                RCLCPP_INFO(logger_, "DBUS Hex: %s", hex_str.c_str());
            }
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }
    }



    // 发送CAN指令
    void command_update() {
        auto builder = board_->start_transmit();

        auto command = motor_.generate_command(control_torque_);

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = 0x200,
                .can_data = device::CanPacket8{
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                    command,
                    device::CanPacket8::PaddingQuarter{},
                }.as_bytes(),
            });
    }

private:
    rclcpp::Logger logger_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    class Command : public rmcs_executor::Component {
    public:
        explicit Command(MotorTest& mt) : mt_(mt) {}
        void update() override { mt_.command_update(); }
    private:
        MotorTest& mt_;
    };
    std::shared_ptr<Command> command_;

    device::DjiMotor motor_;
    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    controller::pid::PidCalculator speed_pid_;  //速度环PID
    controller::pid::PidCalculator angle_pid_;   // 角度环PID
    filter::LowPassFilter<> velocity_filter_;

    double control_torque_ = 0.0;

};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorTest, rmcs_executor::Component)
