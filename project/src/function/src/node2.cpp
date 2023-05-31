#include <functional>
#include <memory>
#include <cstdlib>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "custom/msg/num.hpp"
#include "custom/srv/req.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<custom::msg::Num>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

  void topic_callback(const custom::msg::Num & msg)  
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Received coordinates: '{" << msg.x << ", "<<msg.y<<", "<<msg.z<<"'}");
    coords_ = {msg.x, msg.y, msg.z};
    if (matrix_) {
      multiply();
    }
  }

  void set_matrix(const std::array<float, 9>& matrix)
  {
    matrix_ = matrix;
    if (coords_) {
      multiply();
    }
  }

private:
  void multiply() const
  {
    std::array<float, 3> result = {0};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (*coords_)[j] * (*matrix_)[i * 3 + j];
      }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Transformed coordinates: '{" << result[0] << ", "<<result[1]<<", "<<result[2]<<"}'");
  }

  rclcpp::Subscription<custom::msg::Num>::SharedPtr subscription_;
  std::optional<std::array<float, 3>> coords_;
  std::optional<std::array<float, 9>> matrix_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("minimal_client");
  auto client = node->create_client<custom::srv::Req>("req");

  //if (argc != 2) {
   // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: path JSON");
  //  return 1;
  //}

  auto subscriber = std::make_shared<MinimalSubscriber>();

  auto request = std::make_shared<custom::srv::Req::Request>();
  //request->json_path = std::string(argv[1]);    this is to take the location of the matrix as an argument
  request->json_path = "matrix/matrix.json";  //however i didn't know how to do this from the launch file but it works 
						//without the launch file by typing it into the command line
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto matrix_vector = result.get()->matrix;
    std::array<float, 9> matrix_array;
    for (int i = 0; i < 9; ++i) {
      matrix_array[i] = matrix_vector[i];
    }
    subscriber->set_matrix(matrix_array);
  }
  else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service req.");
    return 1;
  }

rclcpp::spin(subscriber);
rclcpp::shutdown();
return 0;
}
