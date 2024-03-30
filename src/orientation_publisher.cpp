#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cmps12/cmps12_i2c.hpp"



class OrientationPublisherNode : public rclcpp::Node
{
public:
    OrientationPublisherNode() : Node("orientation_publisher_node")
    {
        cmps12_file = cmps12_init("/dev/i2c-1");
        if (cmps12_file < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CMPS12 device");
            throw std::runtime_error("Failed to initialize CMPS12 device");
        }

        orientation_pub = this->create_publisher<sensor_msgs::msg::Imu>("orientation", 10);
        timer_ = this-> create_wall_timer(std::chrono::microseconds(500), std::bind(&OrientationPublisherNode::publish_orientation, this));

    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr orientation_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    int cmps12_file;

    void publish_orientation()
    {

        Orientation orientation_data = cmps12_read_orientation_BNO055(cmps12_file);

        //publish data as IMU message
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.orientation.x = orientation_data.roll;
        imu_msg.orientation.y = orientation_data.pitch;
        imu_msg.orientation.z = orientation_data.bearing;
        imu_msg.orientation.w = 0.0;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "base_link";

        orientation_pub ->publish(imu_msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrientationPublisherNode>());
    rclcpp::shutdown();

    return 0;
}