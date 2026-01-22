#include <chrono>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Function to read a single key without pressing Enter
char getKey()
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");
    return buf;
}

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Teleop node started. Use keys W/A/S/D to move, X to stop.");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TeleopNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        char c = getKey();
        geometry_msgs::msg::Twist msg;
        double linear = 0.2;    // forward/backward speed
        double angular = 1.0;   // rotation speed

        switch(c)
        {
            case 'w': msg.linear.x = linear; break;      // forward
            case 's': msg.linear.x = -linear; break;     // backward
            case 'a': msg.angular.z = angular; break;    // turn left
            case 'd': msg.angular.z = -angular; break;   // turn right
            case 'x': msg.linear.x = 0; msg.angular.z = 0; break; // stop
            default: return;  // ignore other keys
        }

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
