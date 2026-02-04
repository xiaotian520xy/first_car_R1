cmake_minimum_required(VERSION 3.8)
project(chuankou)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(serial REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(send src/send.cpp)
ament_target_dependencies(send
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  serial  # Link against the serial library
)

add_executable(receive src/receive.cpp)
ament_target_dependencies(receive
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  serial  # Link against the serial library
)

# Install executables and libraries
install(TARGETS
  send
  receive
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
