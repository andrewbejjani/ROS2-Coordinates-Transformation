#include <fstream>
#include <string>

#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "custom/srv/req.hpp"

#include <memory>

using json = nlohmann::json;

void mat(const std::shared_ptr<custom::srv::Req::Request> request,     
          std::shared_ptr<custom::srv::Req::Response> response)  
{
  std::ifstream file(request->json_path);
  if (!file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file: %s", request->json_path.c_str());
      return;
    }
  json j =json::parse(file);
  
  
  auto matrix_j = j["matrix"];
  response->matrix.resize(9);
  
   if (!matrix_j.is_array() || matrix_j.size() != 3 || !matrix_j[0].is_array() || matrix_j[0].size() != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid matrix format in file: %s", request->json_path.c_str());
        return;
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (!matrix_j[i][j].is_number_float()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid matrix element format in file: %s", request->json_path.c_str());
                return;
            }
            response->matrix[i * 3 + j] = matrix_j[i][j];
        }
    }
    
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("req_server");   

  rclcpp::Service<custom::srv::Req>::SharedPtr service =               
    node->create_service<custom::srv::Req>("req",  &mat);   

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve matrix"); 

  rclcpp::spin(node);
  rclcpp::shutdown();
}
