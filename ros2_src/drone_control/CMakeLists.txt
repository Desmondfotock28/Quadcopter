cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)

# MAVROS2 motor test
add_executable(motor_test_mavros2 src/motor_test_mavros2.cpp)
target_include_directories(motor_test_mavros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motor_test_mavros2 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  motor_test_mavros2
  "rclcpp"
  "mavros_msgs"
)

install(TARGETS motor_test_mavros2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
