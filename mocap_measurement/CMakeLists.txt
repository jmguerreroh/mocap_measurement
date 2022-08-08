cmake_minimum_required(VERSION 3.5)
project(mocap_measurement)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap_measurement_msgs REQUIRED)

# OpenCV node
add_executable(measurement src/measurement.cpp)
target_include_directories(measurement PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(measurement PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  measurement
  "rclcpp"
  "mocap_msgs"
  "geometry_msgs"
  "mocap_measurement_msgs"
)

install(TARGETS measurement
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()