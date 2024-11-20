#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
#include <iostream>

class ArduinoSerialNode : public rclcpp::Node {

	
public:
	ArduinoSerialNode() : Node("arduino_serial_node") {
		serial_port_.Open("/dev/ttyACM0");
		serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

		if (serial_port_.IsOpen()) {
			RCLCPP_INFO(this->get_logger(), "Serial connection established");
		}

		timer_ = this->create_wall_timer(
				std::chrono::seconds(1),
				std::bind(&ArduinoSerialNode::readSerialData, this));

		// Additional timer for sending data
		send_timer_ = this->create_wall_timer(
				std::chrono::seconds(2),
				std::bind(&ArduinoSerialNode::sendSerialData, this));
				
	}

private:
	void readSerialData() {
		std::string data;
		if (serial_port_.IsDataAvailable()) {
			serial_port_.ReadLine(data);
			RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());
		}
	}

	void sendSerialData() {
		std::string message = "Hello from Raspberry Pi\n";
		serial_port_.Write(message);
		RCLCPP_INFO(this->get_logger(), "Sent: %s", message.c_str());
	}


	LibSerial::SerialPort serial_port_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr send_timer_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArduinoSerialNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
