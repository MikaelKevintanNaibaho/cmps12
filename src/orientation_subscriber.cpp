#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cmps12/cmps12_i2c.hpp"

class OrientationSubscribeNode : public rclcpp::Node
{
public:
    OrientationSubscribeNode() : Node("Orientation_subscriber_node")
    {
        orientation_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "orientation", 10, std::bind(&OrientationSubscribeNode::orientation_callback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub;

    void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Recieved Orientation: ");
        RCLCPP_INFO(this->get_logger(), "Roll: %f", msg->orientation.x);
        RCLCPP_INFO(this->get_logger(), "Pitch: %f", msg->orientation.z);
        RCLCPP_INFO(this->get_logger(), "Bearing: %f", msg->orientation.x);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrientationSubscribeNode>());
    rclcpp::shutdown();
    return 0;
}