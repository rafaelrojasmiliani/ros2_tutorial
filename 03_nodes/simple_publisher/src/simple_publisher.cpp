
#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker") {

    msg_.data = "message";

    auto publish_message = [this]() -> void {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_.data.c_str());
      pub_->publish(msg_);
    };

    pub_ = this->create_publisher<std_msgs::msg::String>(
        "chatter", rclcpp::QoS(rclcpp::KeepLast(7)));

    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  std_msgs::msg::String msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
class Listener : public rclcpp::Node {
public:
  Listener() : Node("listener") {

    auto callback = [this](const std_msgs::msg::String::SharedPtr _msg) {
      RCLCPP_INFO(this->get_logger(), "Message arrived: '%s'",
                  _msg->data.c_str());
    };

    sub_data_ = this->create_subscription<std_msgs::msg::String>("chatter", 10,
                                                                 callback);
  }

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;
};

int main(int argc, char **argv) {
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  Talker::SharedPtr tal = std::make_shared<Talker>();
  Listener::SharedPtr lis = std::make_shared<Listener>();
  executor.add_node(tal);
  executor.add_node(lis);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
