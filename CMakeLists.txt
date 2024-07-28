cmake_minimum_required(VERSION 3.8)
project(ultra_sonic_radar_driver)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_executable(ultra_sonic_radar_driver_node
  src/ultra_sonic_radar_driver.cpp
  src/ultra_sonic_radar_driver_main.cpp
)

# install

install(
  PROGRAMS
  test/configure_radar.sh
  test/simulate_can_send.sh
  DESTINATION 
  lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)