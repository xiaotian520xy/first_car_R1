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
find_package(fyt_msg REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 添加头文件搜索路径 - 注意这里包含了chuankou子目录的父目录
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# 或者使用target_include_directories（更现代的方式）
add_executable(send src/send.cpp)
ament_target_dependencies(send
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  serial
)

# 为send指定头文件目录
target_include_directories(send PRIVATE
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_executable(receive src/receive.cpp src/autopath.cpp)
ament_target_dependencies(receive
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  fyt_msg
  serial
)

# 为receive指定头文件目录
target_include_directories(receive PRIVATE
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Install executables
install(TARGETS
  send
  receive
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（可选）
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()