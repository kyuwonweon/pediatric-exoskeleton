cmake_minimum_required(VERSION 3.8)
project(micropython_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCANActivity.srv"
)

# rosidl already installs the micropython_logger Python package (for .srv), so a
# second ament_python_install_package would collide; add our modules to it instead.
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__init__.py" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)
install(PROGRAMS
  scripts/multi_robot.py
  scripts/ROS2tcp.py
  scripts/ROS2udp.py
  scripts/tcp2ROS.py
  scripts/udp2ROS.py
  scripts/canIP2ROS.py
  scripts/ROS2canIP.py
  scripts/CANMotorGroup.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)
install(DIRECTORY cubemars_sm_params sm_multi_act_params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_can_sm_profiles tests/test_can_sm_profiles.py)
  ament_add_pytest_test(test_can_motor_group tests/test_can_motor_group.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()