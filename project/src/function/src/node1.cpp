#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom/msg/num.hpp"                                            

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<custom::msg::Num>("topic", 10);  
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = custom::msg::Num();                                  
    message.x = 1.5;
    message.y = 2.0;
    message.z = 1.0;                                                     
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '"<<message.x<<", "<<message.y<<", "<<message.z <<"'");    // CHANGE
    publisher_->publish(message);
    timer_->cancel();
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom::msg::Num>::SharedPtr publisher_;             
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
