#include <chrono>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

using namespace std::chrono_literals;

class MotorTestMavros : public rclcpp::Node
{
public:
    MotorTestMavros() : Node("motor_test_mavros")
    {
        RCLCPP_INFO(this->get_logger(), "MAVROS2 Motor Test Node Starting...");
        
        // Publishers
        rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            "/mavros/rc/override", 10);
        
        // Subscribers
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&MotorTestMavros::state_callback, this, std::placeholders::_1));
        
        // Service clients
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        
        // Timer for the test sequence
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MotorTestMavros::timer_callback, this));
        
        start_time_ = this->get_clock()->now();
        
        // Initialize RC values (neutral positions, throttle low)
        rc_msg_.channels = {1500, 1500, 1100, 1500, 1500, 1500, 1500, 1500, 
                           65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535};
    }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        if (!connection_logged_ && current_state_.connected) {
            RCLCPP_INFO(this->get_logger(), "FCU connected!");
            connection_logged_ = true;
        }
    }
    
    void timer_callback()
    {
        auto current_time = this->get_clock()->now();
        auto elapsed = (current_time - start_time_).seconds();
        
        // Wait for FCU connection
        if (!current_state_.connected) {
            if (elapsed > 1.0 && !waiting_logged_) {
                RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
                waiting_logged_ = true;
            }
            return;
        }
        
        // Test sequence
        if (elapsed < 2.0) {
            // Wait 2 seconds after connection
            if (!wait_logged_) {
                RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds before arming...");
                wait_logged_ = true;
            }
        } else if (elapsed < 3.0) {
            // Arm the drone
            if (!arm_attempted_) {
                RCLCPP_INFO(this->get_logger(), "Attempting to arm...");
                arm_drone();
                arm_attempted_ = true;
            }
        } else if (elapsed < 5.0) {
            // Motors on for 2 seconds
            if (current_state_.armed) {
                rc_msg_.channels[2] = 1200; // Throttle channel (low speed)
                rc_override_pub_->publish(rc_msg_);
                
                if (!motors_on_logged_) {
                    RCLCPP_INFO(this->get_logger(), "Motors ON (throttle: 1200)");
                    motors_on_logged_ = true;
                }
            }
        } else if (elapsed < 5.5) {
            // Motors off
            rc_msg_.channels[2] = 1100; // Throttle to minimum
            rc_override_pub_->publish(rc_msg_);
            
            if (!motors_off_logged_) {
                RCLCPP_INFO(this->get_logger(), "Motors OFF (throttle: 1100)");
                motors_off_logged_ = true;
            }
        } else {
            // Disarm and shutdown
            if (!disarm_attempted_) {
                RCLCPP_INFO(this->get_logger(), "Disarming...");
                disarm_drone();
                disarm_attempted_ = true;
            }
            
            if (elapsed > 6.0) {
                RCLCPP_INFO(this->get_logger(), "Test complete. Shutting down.");
                timer_->cancel();
                rclcpp::shutdown();
            }
        }
    }
    
    void arm_drone()
    {
        if (!arming_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available");
            return;
        }
        
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        
        auto future = arming_client_->async_send_request(request);
        
        // Non-blocking check
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 1s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Armed successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Arming failed with result: %d", result->result);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Arming service call timed out");
        }
    }
    
    void disarm_drone()
    {
        if (!arming_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available");
            return;
        }
        
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;
        
        auto future = arming_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 1s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Disarmed successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Disarming failed with result: %d", result->result);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Disarming service call timed out");
        }
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    
    // Service clients
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    
    // State
    mavros_msgs::msg::State current_state_;
    mavros_msgs::msg::OverrideRCIn rc_msg_;
    
    // Flags
    bool connection_logged_ = false;
    bool waiting_logged_ = false;
    bool wait_logged_ = false;
    bool arm_attempted_ = false;
    bool motors_on_logged_ = false;
    bool motors_off_logged_ = false;
    bool disarm_attempted_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        rclcpp::spin(std::make_shared<MotorTestMavros>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_test_mavros"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}