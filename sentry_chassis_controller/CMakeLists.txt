cmake_minimum_required(VERSION 3.0.2)
project(sentry_chassis_controller)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  std_msgs
  geometry_msgs
  tf
  nav_msgs
  dynamic_reconfigure
)
generate_dynamic_reconfigure_options(
  config/sentry_chassis_controller.cfg
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS control_toolbox controller_interface hardware_interface pluginlib roscpp std_msgs geometry_msgs nav_msgs dynamic_reconfigure
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/sentry_chassis_controller.cpp
  src/kinematics.cpp
  src/test_pid.cpp
  src/odometry.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# --- 键盘监听节点 ---
add_executable(choose_testmode_node src/choose_testmode.cpp)
target_link_libraries(choose_testmode_node ${catkin_LIBRARIES})