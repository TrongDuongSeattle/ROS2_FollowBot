#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <libserial/SerialPort.h>
#include <sstream>
#include <nlohmann/json.hpp>

class GPSSerialNode : public rclcpp::Node {
 public:
	 GPSSerialNode() : Node("gps_serial_node") {
		serial_port_.Open("/dev/ttyACM0");
		serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

		if (serial_port_.IsOpen()) 
			RCLCPP_INFO(this->get_logger(), "Serial connection established.");
		else 
			RCLCPP_ERROR(this->get_logger(), "Failed to open serial connection.");

		gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
			"gps/fix", 10);

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100), // read IMU data every 100ms
			std::bind(&GPSSerialNode::readGPSData, this)		
		);
	 }

 private:
	 void readGPSData() {
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
				if (json_msg["sensor_type"] != "gps") {
					RCLCPP_WARN(this->get_logger(), "Ignoring non-GPS message");
					return;
				}

				double latitude = json_msg["data"]["latitude"];
				double longitude = json_msg["data"]["longitude"];

				sensor_msgs::msg::NavSatFix gps_msg;
				gps_msg.header.stamp = this->now();
				gps_msg.header.frame_id = "gps_link";

				gps_msg.latitude = latitude;
				gps_msg.longitude = longitude;

				// default covariance
				for (size_t i = 0; i < 9; i++)
					gps_msg.position_covariance[i] = 1.0;

				gps_publisher_->publish(gps_msg);
				serial_port_.Write("Successfully published GPS data to `gps/fix`");

			} catch (const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "JSON Parsing Error: %s", e.what());
			}
		}
	 }

	 LibSerial::SerialPort serial_port_;
	 rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
	 rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GPSSerialNode>());
	rclcpp::shutdown();
	return 0;
}
