#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

//ESTOS NODOS SOLAMENTE SON PARA DEBUG

using namespace std::chrono_literals;

// Listener node (subscriber)
class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode() : Node("listener_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "saludo", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Mensaje recibido: '%s'", msg->data.c_str());
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto listener_node = std::make_shared<ListenerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(listener_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

