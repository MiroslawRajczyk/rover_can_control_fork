cmake_minimum_required(VERSION 3.0.2)
project(rover_can_control_fork)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tools
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_can_board_driver src/test_can_board_driver.cpp src/test_can_board.cpp src/test_can_boards.cpp)
target_link_libraries(test_can_board_driver ${catkin_LIBRARIES})

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
  scripts/drive.py
  scripts/manipulator_can_motors_drive.py
  scripts/status_led_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
