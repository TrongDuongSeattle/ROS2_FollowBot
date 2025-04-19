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
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)     # for encoder
find_package(sensor_msgs REQUIRED)  # for imu and gps
find_package(PkgConfig REQUIRED)
find_package(nlohmann_json REQUIRED)

pkg_check_modules(LibSerial REQUIRED IMPORTED_TARGET libserial)
include_directories(${LibSerial_INCLUDE_DIRS})

# == Serial Manager Component ==
add_library(serial_manager_component SHARED src/serial_manager_node.cpp)
target_link_libraries(serial_manager_component PkgConfig::LibSerial)
ament_target_dependencies(serial_manager_component
    rclcpp
    rclcpp_components
    nlohmann_json
)
target_include_directories(serial_manager_component PUBLIC src/include)
rclcpp_components_register_node(serial_manager_component
    PLUGIN "SerialManagerNode"
    EXECUTABLE serial_manager_node_exec
)

# ======= IMU component =======
add_library(madgwick_filter src/imu_filter/MadgwickAHRS.cpp)
target_include_directories(madgwick_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/imu_filter) 

add_library(imu_serial_component SHARED src/imu_serial_node.cpp)
target_link_libraries(imu_serial_component 
	madgwick_filter 
	PkgConfig::LibSerial
)
ament_target_dependencies(imu_serial_component 
	rclcpp 
	rclcpp_components 
	sensor_msgs
	nlohmann_json	
) 
target_include_directories(imu_serial_component PUBLIC src/include)

rclcpp_components_register_node(
	imu_serial_component
	PLUGIN "IMUSerialNode"
	EXECUTABLE imu_serial_node_exec
)

# ===== encoder component =====
add_library(encoder_serial_component SHARED src/encoder_serial_node.cpp)
target_link_libraries(encoder_serial_component PkgConfig::LibSerial)
ament_target_dependencies(encoder_serial_component 
	rclcpp 
	rclcpp_components 
	nav_msgs
	nlohmann_json
)
target_include_directories(encoder_serial_component PUBLIC src/include)

rclcpp_components_register_node(
	encoder_serial_component
	PLUGIN "EncoderSerialNode"
	EXECUTABLE encoder_serial_node_exec
)

# ======= gps component =======
add_library(gps_serial_component SHARED src/gps_serial_node.cpp)
target_link_libraries(gps_serial_component PkgConfig::LibSerial)
ament_target_dependencies(gps_serial_component 
	rclcpp 
	rclcpp_components 
	sensor_msgs 
	nlohmann_json)
target_include_directories(gps_serial_component PUBLIC src/include)

rclcpp_components_register_node(
	gps_serial_component
	PLUGIN "GPSSerialNode"
	EXECUTABLE gps_serial_node_exec
)

install(
	TARGETS serial_manager_component
			serial_manager_node_exec
			imu_serial_component 
			imu_serial_node_exec
			encoder_serial_component
			encoder_serial_node_exec	
			gps_serial_component
			gps_serial_node_exec
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
	rclcpp 
	rclcpp_components
	nav_msgs
	sensor_msgs
	nlohmann_json
)
ament_package()
