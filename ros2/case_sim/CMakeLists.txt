cmake_minimum_required(VERSION 3.16)
project(case_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(case_sim_controller SHARED src/balance_controller.cpp)
target_include_directories(case_sim_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_compile_features(case_sim_controller PUBLIC cxx_std_17)
ament_target_dependencies(case_sim_controller
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs)

pluginlib_export_plugin_description_file(controller_interface case_sim_plugins.xml)

install(TARGETS case_sim_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/sweep.py
  scripts/analyze.py
  scripts/run_supervisor.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Equivalence test: the vendored upstream PID_v1 is compiled against a stubbed
  # clock and driven alongside the port. Any drift in the port fails here, and
  # every sweep result rests on the port being exact.
  ament_add_gtest(test_pid_v1_equivalence
    test/test_pid_v1_equivalence.cpp
    test/vendor/PID_v1.cpp)
  target_compile_features(test_pid_v1_equivalence PRIVATE cxx_std_17)
  target_include_directories(test_pid_v1_equivalence PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test/vendor)

  ament_add_gtest(test_motor_model test/test_motor_model.cpp)
  target_compile_features(test_motor_model PRIVATE cxx_std_17)
  target_include_directories(test_motor_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

ament_package()
