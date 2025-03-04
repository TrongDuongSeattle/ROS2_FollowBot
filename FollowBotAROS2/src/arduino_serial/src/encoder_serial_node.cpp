#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <libserial/SerialPort.h>
#include <nlohmann/json.hpp>

class EncoderSerialNode : public rclcpp::Node {
 public:
	 EncoderSerialNode() : Node("encoder_serial_node") {
		serial_port_.Open("/dev/ttyAMC0");
		serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

		if (serial_port_.IsOpen())
			RCLCPP_INFO(this->get_logger(), "Serial connection established.");

		odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("encoder/data", 10);

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(50);
			std::bind(&EncoderSerialNode::readEncoderData, this);		
		);

		prev_time_ = this->get_clock()->now();
	 }

 private:
	 void readEncoderData() {
		std::string data;
		if (serial_port_.IsDataAvailable()) {
			serial_port_.ReadLine(data);
			RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());
		
			try {
				auto json_msg = nlohmann::json::parse(data);
				if (json_msg["sensor_type"] != "encoder") {
					RCLCPP_WARN(this->get_logger(), "Ignoring non-encoder message");
					return;
				}

				rclcpp::Time current_time = this->get_clock()->now();
				double dt = (current_time - prev_time_).seconds();
				prev_time_ = current_time_;

				double left_ticks = json_msg["data"]["left_wheel_ticks"];
				double right_ticks = json_msg["data"]["right_wheel_ticks"];
				double wheel_radius = json_msg["data"]["wheel_radius"];
				double wheel_base = json_msg["data"]["wheel_base"];

				double left_velocity = (left_wheel_ticks * wheel_radius) / dt;
				double right_velocity = (right_wheel_ticks * wheel_radius) / dt;
				
				double linear_velocity = (left_velocity + right_velocity) / 2.0;
				double angular_velocity = (right_velocity - left_velocity) / wheel_base;

				nav_msgs::msg::Odometry odom_msg;
				odom_msg.header.stamp = current_time;
				odom_msg.header.frame_id = "odom";
				odom_msg.child_frame_id = "base_link";

				odom_msg.twist.twist.linear.x = linear_velocity;
				odom_msg.twist.twist.angular.z = angular_velocity; // other source for yaw

				// covariance (adjust based on sensor noise);
				odom_msg.twist.covariance[0] = 0.01;  // uncertainty in linear velocity
                odom_msg.twist.covariance[35] = 0.01; // uncertain in angular velocity


				odom_publisher_->publish(odom_msg);
			} catch (const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "JSON Parsing Error: %s", e.what());
			}
		 }
	 }

	 LibSerial::SerialPort serial_port_;
	 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
	 rclcpp::TimerBase::SharedPtr timer_;
	 rclcpp::Time prev_time_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EncoderSerialNode>());
	rclcpp::shutdown();
	return 0;

}
