cmake_minimum_required(VERSION 3.8)
project(maze_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf worlds DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/common.py
  scripts/strategies.py
  scripts/base.py
  scripts/wall_map.py
  scripts/explorer.py
  scripts/color_sensor.py
  scripts/calibrate_color.py
  scripts/aruco_detector.py
  scripts/display.py
  scripts/joy_drive.py
  scripts/motors.py
  scripts/imu.py
  scripts/tcs34725.py
  scripts/drive_test.py
  scripts/pista_b_track.py
  scripts/pista_b_map.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(strategies test/test_strategies.py)
  ament_add_pytest_test(display test/test_display.py)
  ament_add_pytest_test(common test/test_common.py)
  ament_add_pytest_test(sensors test/test_sensors.py)
  ament_add_pytest_test(joy test/test_joy.py)
endif()

ament_package()
