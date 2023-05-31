cmake_minimum_required(VERSION 3.8)
project(function)

find_package(nlohmann_json REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom REQUIRED)                      

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${custom_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

add_executable(talker src/node1.cpp)
ament_target_dependencies(talker rclcpp custom)  

add_executable(listener_client src/node2.cpp)
ament_target_dependencies(listener_client rclcpp custom) 

add_executable(server src/node3.cpp)
ament_target_dependencies(server rclcpp custom nlohmann_json) 


target_link_libraries(server
    nlohmann_json::nlohmann_json
)

install(TARGETS
  talker
  listener_client
  server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
