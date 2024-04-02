#include "cr_hw_interface/light_sub.hpp"


using namespace light_sub;

CrCommunication::CrCommunication():Node("cr_hw_comm")
{
    this->LED_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("camera_led", rclcpp::SystemDefaultsQoS(), std::bind(&CrCommunication::led_callback, this, std::placeholders::_1));
    this->led_value_ = 0;  // init light off
}


void CrCommunication::led_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    this->led_value_ = int(msg->data);
    RCLCPP_INFO(this->get_logger(), "Setting camera LEDs to : '%i'", int(msg->data));
}

int CrCommunication::getLEDValue()
{
    return this->led_value_;
}
