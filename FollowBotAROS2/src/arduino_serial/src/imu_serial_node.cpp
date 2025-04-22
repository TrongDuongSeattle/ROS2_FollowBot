#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_filter/MadgwickAHRS.h"
#include <nlohmann/json.hpp>
#include "serial_manager.hpp"

#include <chrono>
#include <functional> // for std::bind
#include <memory>
#include <string>

class IMUSerialNode : public rclcpp::Node {
 public:
	 IMUSerialNode(const rclcpp::NodeOptions & options) 
	 : Node("imu_serial_node", options), madgwick_filter() {
		
		SerialManager::get().open_port("/dev/ttyACM0");

		imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
			"imu/data", 10);

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100), // read IMU data every 100ms
			std::bind(&IMUSerialNode::readIMUData, this)		
		);
	 }

 private:
	 void readIMUData() {
		auto json_msg = SerialManager::get().popImu();
		
		if (!json_msg.has_value())
			return;

		try {
			RCLCPP_INFO(this->get_logger(), "Processing IMU data");
			
			auto& data = json_msg->at("data");

			double ax = data["ax"].get<double>();
			double ay = data["ay"].get<double>();
			double az = data["az"].get<double>();
			double gx = data["gx"].get<double>();
			double gy = data["gy"].get<double>();
			double gz = data["gz"].get<double>();

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

			imu_msg.orientation_covariance[0] = 0.001; // variance in x
			imu_msg.orientation_covariance[4] = 0.001; // variance in y
			imu_msg.orientation_covariance[8] = 0.001; // variance in z
			
			imu_msg.angular_velocity_covariance[0] = 0.01;
			imu_msg.angular_velocity_covariance[4] = 0.01;
			imu_msg.angular_velocity_covariance[8] = 0.01;

			imu_msg.linear_acceleration_covariance[0] = 0.1;
			imu_msg.linear_acceleration_covariance[4] = 0.1;
			imu_msg.linear_acceleration_covariance[8] = 0.1;

			imu_publisher_->publish(imu_msg);
			RCLCPP_INFO(this->get_logger(), 
				"Successfully published IMU data to imu/data");

		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "IMU error: %s", e.what());
		}
	 }

	 rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
	 rclcpp::TimerBase::SharedPtr timer_;
	 Madgwick madgwick_filter;
};


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(IMUSerialNode)
