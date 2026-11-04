cmake_minimum_required(VERSION 3.8)
project(robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

install(
  DIRECTORY urdf launch worlds rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/teleop_wasd.py
    scripts/benchmark_ate.py
    scripts/benchmark_system.py
    scripts/nav2_goal_tester.py
    scripts/pid_velocity_controller.py
    scripts/dummy_robot.py
    scripts/webcam_rgbd_node.py
    scripts/intelligent_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
