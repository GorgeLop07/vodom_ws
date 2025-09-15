#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;


//ESTOS NODOS SOLAMENTE SON PARA DEBUG
//nomas hola, pero varias veces


class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("vodom_first_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("saludo", 10);
        timer_ = this->create_wall_timer(
            500ms, [this]() {
                auto message = std_msgs::msg::String();
                message.data = "Hola, este es un mensaje publicado constantemente";
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Mensaje publicado: '%s'", message.data.c_str());
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto publisher_node = std::make_shared<MyNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(publisher_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

