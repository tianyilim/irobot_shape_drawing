cmake_minimum_required(VERSION 3.8)
project(irobot_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executables here
add_executable(undock_client src/undock_client.cpp)
ament_target_dependencies(undock_client rclcpp rclcpp_action irobot_create_msgs)

add_executable(trace_seven src/trace_seven.cpp)
ament_target_dependencies(trace_seven rclcpp rclcpp_action irobot_create_msgs std_msgs geometry_msgs tf2 tf2_ros)

add_executable(trace_s src/trace_s.cpp)
ament_target_dependencies(trace_s rclcpp rclcpp_action irobot_create_msgs std_msgs geometry_msgs tf2 tf2_ros)

# And here
install(TARGETS
  undock_client
  trace_seven
  trace_s
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
