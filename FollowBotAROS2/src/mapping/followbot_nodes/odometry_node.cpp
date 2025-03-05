#include <rclcpp/rclcpp.hpp>         // ROS2 C++ API
#include <sensor_msgs/msg/imu.hpp>   // IMU message type
#include <nav_msgs/msg/odometry.hpp> // odom message type 

#include <geometry_msgs/msg/transform_stamped.hpp> // transform message type
#include <tf2_ros/transform_broadcaster.h>         // transfrm message broadcaster for RVIZ

#include <chrono>   // time functions 
#include <memory>   // smart pointers
#include <cmath>    // math functions

// Odometry Publisher class, inheriting from rclcpp::Node
class OdometryPublisher : public rclcpp::Node {
 public:
	OdometryPublisher() 
		: Node("followbot_odom"), 
		x_(0.0), y_(0.0), theta_(0.0), 
		linear_velocity_(0.0), angular_velocity_(0.0) 
	{
		odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
		
		imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
			"imu/data", 10, 
			std::bind(&OdometryPublisher::imu_callback, this, std::placeholders::_1)
		);

		encoder_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"encoder/data", 10,
			std::bind(&OdometryPublisher::encoder_callback, this, std::placeholders::_1)	
		);
		
		// timer for publishing odom at regular intervals
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100), 
			std::bind(&OdometryPublisher::update_odom, this)
		);
		
		// transform broadcaster for RViz visualization
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
		
		prev_time_ = this->get_clock()->now();

		RCLCPP_INFO(this->get_logger(), "Odometry Node initialized.");
	}

 private:
	void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
		orientation_ = msg->orientation;
		angular_velocity_ = msg->angular_velocity.z; // yaw
	}

	void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		linear_velocity_ = msg->twist.twist.linear.x;
	}

	void update_odom() {
		rclcpp::Time current_time = this->get_clock()->now();
		double deltaTime = (current_time - prev_time_).seconds();
		prev_time_ = current_time;

		theta_ += angular_velocity_ * deltaTime;
		x_ += linear_velocity_ * deltaTime * cos(theta_);
		y_ += linear_velocity_ * deltaTime * sin(theta_);

		auto odom_msg = nav_msgs::msg::Odometry();
		odom_msg.header.stamp = this->get_clock()->now();
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";

		odom_msg.pose.pose.position.x = x_;
		odom_msg.pose.pose.position.y = y_;
		odom_msg.pose.pose.orientation = orientation_;

		odom_msg.twist.twist.linear.x = linear_velocity_;
		odom_msg.twist.twist.angular.z = angular_velocity_;

		odom_publisher_->publish(odom_msg);

		// broadcast transform from odom to base_link
		geometry_msgs::msg::TransformStamped t;
		t.header.stamp = this->get_clock()->now();
		t.header.frame_id = "odom";
		t.child_frame_id = "base_link";
		t.transform.translation.x = x_;
		t.transform.translation.y = y_;
		t.transform.translation.z = 0.0;
		t.transform.rotation = orientation_;

		tf_broadcaster_->sendTransform(t);
	}

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_subscriber_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	rclcpp::Time prev_time_;
	geometry_msgs::msg::Quaternion orientation_;
	double x_, y_, theta_;
	double linear_velocity_, angular_velocity_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdometryPublisher>());
	rclcpp::shutdown();
	return 0;
}

