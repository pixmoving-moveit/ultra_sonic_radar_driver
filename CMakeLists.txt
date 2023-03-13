cmake_minimum_required(VERSION 3.0.2)
project(ultra_sonic_radar_driver)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pix_driver
  CATKIN_DEPENDS can_msgs roscpp std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/ultra_sonic_radar_driver.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# src/steering_command_102.cc
# src/{name}.cc
add_executable(${PROJECT_NAME}_node
  src/ultra_sonic_radar_driver_main.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS scripts/fake_radar_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# catkin_install_python(PROGRAMS
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
