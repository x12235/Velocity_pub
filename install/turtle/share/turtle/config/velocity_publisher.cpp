#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher()
    : Node("velocity_publisher") // Node name initialization
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/velocity_controller/cmd_vel", 10); // Creating the publisher

        timer_ = this->create_wall_timer(
            50ms, std::bind(&VelocityPublisher::publishCommand, this)); // Timer set to 50 ms interval
    }

private:
    void publishCommand() // Sends linear velocity
    {
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = this->now();
        command.twist.linear.x = 0.1;
        command.twist.angular.z = 0.0; // Ensure correct assignment to the angular component
        publisher_->publish(command);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}
