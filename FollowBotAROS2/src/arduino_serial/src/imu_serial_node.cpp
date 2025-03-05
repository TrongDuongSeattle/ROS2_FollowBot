#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <libserial/SerialPort.h>
#include "MadgwickAHRS.h"
#include <iostream>
#include <sstream>
#include <nlohmann/json.hpp>

class IMUSerialNode : public rclcpp::Node {
 public:
	 IMUSerialNode() : Node("imu_serial_node"), madgwick_filter() {
		serial_port_.Open("/dev/ttyACM0");
		serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

		if (serial_port_.IsOpen()) 
			RCLCPP_INFO(this->get_logger(), "Serial connection established.");

		imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(50), // read IMU data every 50ms
			std::bind(&IMUSerialNode::readIMUData, this)		
		);
	 }

 private:
	 void readIMUData() {
		std::string data;
		if (serial_port_.IsDataAvailable()) {
			serial_port_.ReadLine(data);
			RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());

			if (data.empty() || data.front() != '{') {
				RCLCPP_WARN(this->get_logger(), 
					"Ignoring non-JSON message: %s", data.c_str());
				return;
			}

			try {
				auto json_msg = nlohmann::json::parse(data);
				if (json_msg["sensor_type"] != "imu") {
					RCLCPP_WARN(this->get_logger(), "Ignoring non-IMU message");
					return;
				}

				double ax = json_msg["data"]["ax"];
				double ay = json_msg["data"]["ay"];
				double az = json_msg["data"]["az"];
				double gx = json_msg["data"]["gx"];
				double gy = json_msg["data"]["gy"];
				double gz = json_msg["data"]["gz"];

				sensor_msgs::msg::Imu imu_msg;
				imu_msg.header.stamp = this->now();
				imu_msg.header.frame_id = "imu_link";

				madgwick_filter.updateIMU(gx, gy, gz, ax, ay, az);

				imu_msg.linear_acceleration.x = ax;
				imu_msg.linear_acceleration.y = ay;
				imu_msg.linear_acceleration.z = az;

				imu_msg.angular_velocity.x = gx;
				imu_msg.angular_velocity.y = gy;
				imu_msg.angular_velocity.z = gz;

				imu_msg.orientation.x = madgwick_filter.getQuaternionX();
				imu_msg.orientation.y = madgwick_filter.getQuaternionY();
				imu_msg.orientation.z = madgwick_filter.getQuaternionZ();
				imu_msg.orientation.w = madgwick_filter.getQuaternionW();

				// Add covariance (required for EKF)
				imu_msg.orientation_covariance[0] = 0.001;
				imu_msg.orientation_covariance[4] = 0.001;
				imu_msg.orientation_covariance[8] = 0.001;
				
				imu_msg.angular_velocity_covariance[0] = 0.01;
				imu_msg.angular_velocity_covariance[4] = 0.01;
				imu_msg.angular_velocity_covariance[8] = 0.01;

				imu_msg.linear_acceleration_covariance[0] = 0.1;
				imu_msg.linear_acceleration_covariance[4] = 0.1;
				imu_msg.linear_acceleration_covariance[8] = 0.1;

				imu_publisher_->publish(imu_msg);
				serial_port_.Write("Successfully published IMU data to Pi");

			} catch (const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "JSON Parsing Error: %s", e.what());
			}
			/**
			std::stringstream ss(data);
			double ax, ay, az, gx, gy, gz;
			char comma;
			if (ss >> ax >> comma >> ay >> comma >> az >> comma >> 
					gx >> comma >> gy >> comma >> gz) {
				
				madgwick_filter.updateIMU(gx, gy, gz, ax, ay, az);

				imu_msg.linear_acceleration.x = ax;
				imu_msg.linear_acceleration.y = ay;
				imu_msg.linear_acceleration.z = az;
				
				imu_msg.angular_velocity.x = gx;
				imu_msg.angular_velocity.y = gy;
				imu_msg.angular_velocity.z = gz;

				imu_msg.orientation.x = madgwick_filter.getQuaternionX();
				imu_msg.orientation.y = madgwick_filter.getQuaternionY();
				imu_msg.orientation.z = madgwick_filter.getQuaternionZ();
				imu_msg.orientation.w = madgwick_filter.getQuaternionW();

				imu_publisher_->publish(imu_msg);
				serial_port_.Write("Successfuly published IMU data to Pi");
			} else {
				RCLCPP_WARN(this->get_logger(), "Failed to parse IMU data");
			}
			*/
		}
	 }

	 LibSerial::SerialPort serial_port_;
	 rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
	 rclcpp::TimerBase::SharedPtr timer_;
	 Madgwick madgwick_filter;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IMUSerialNode>());
	rclcpp::shutdown();
	return 0;
}
