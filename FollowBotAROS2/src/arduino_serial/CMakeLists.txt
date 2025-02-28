cmake_minimum_required(VERSION 3.8)
project(arduino_serial)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED IMPORTED_TARGET libserial)

# manually add libserial include directory and library path
include_directories(/usr/include/libserial)
link_directories(/usr/lib/aarch64-linux-gnu)

add_library(madgwick_filter src/imu_filter/MadgwickAHRS.cpp)
target_include_directories(madgwick_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/imu_filter) 

add_executable(imu_serial_node src/imu_serial_node.cpp)
target_link_libraries(imu_serial_node madgwick_filter PkgConfig::LibSerial)
ament_target_dependencies(imu_serial_node rclcpp sensor_msgs LibSerial) 

add_executable(arduino_serial_node src/arduino_serial_node.cpp)
target_link_libraries(arduino_serial_node serial)
ament_target_dependencies(arduino_serial_node rclcpp std_msgs)

install(TARGETS
	arduino_serial_node
	DESTINATION lib/${PROJECT_NAME})

ament_package()
