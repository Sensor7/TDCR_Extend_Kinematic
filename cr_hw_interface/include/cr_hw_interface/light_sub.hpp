#ifndef CR_COMM_H
#define CR_COMM_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace light_sub
{
class CrCommunication : public rclcpp::Node
{
    public:
        CrCommunication();

        int getLEDValue();
    private:
        int led_value_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr LED_subscriber_;
        void led_callback(const std_msgs::msg::Int32::SharedPtr msg);

};
}
#endif