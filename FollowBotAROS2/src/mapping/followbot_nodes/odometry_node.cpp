/**
 * The purpose of this class was purely for practice.
 * The functions of this class (sensor fusion and localization) is better 
 * achieved by: robot_localization package and encoder_serial_node.cpp
 */ 
#include <rclcpp/rclcpp.hpp>         // ROS2 C++ API
#include <sensor_msgs/msg/imu.hpp>   // IMU message type
#include <nav_msgs/msg/odometry.hpp> // odom message type 

#include <geometry_msgs/msg/transform_stamped.hpp> // transform message type
#include <tf2_ros/transform_broadcaster.h>         // transfrm message broadcaster for RVIZ
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>   // time functions 
#include <memory>   // smart pointers
#include <cmath>    // math functions

// Odometry Publisher class, inheriting from rclcpp::Node
class OdometryPublisher : public rclcpp::Node {
 public:
	 //changin followbot_odom to odom, matching names
	OdometryPublisher() 
		: Node("odom"), x_(0.0), y_(0.0), theta_(0.0), 
		linear_velocity_(0.0), angular_velocity_(0.0) 
	{
		//Initialization orientation to identity quaternion (no rotation)
		orientation_.w = 1.0;
		orientation_.x = 0.0;
		orientation_.y = 0.0;
		orientation_.z = 0.0;
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
			std::chrono::milliseconds(300), 
			std::bind(&OdometryPublisher::update_odom, this)
		);
		
		// transform broadcaster for RViz visualization
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
		
		prev_time_ = this->get_clock()->now();

		RCLCPP_INFO(this->get_logger(), "Odometry Node initialized.");
	}

 private:  
	void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "IMU");
		
		orientation_ = msg->orientation;
		angular_velocity_ = msg->angular_velocity.z; // yaw

		// Normalize the quarts and ensure valid rotations
		tf2::Quaternion q(
			orientation_.x,
			orientation_.y,
			orientation_.z,
			orientation_.w
		);
		q.normalize();	// Critical! Prevents invalid transforms
		orientation_.x = q.x();
		orientation_.y = q.y();
		orientation_.z = q.z();
		orientation_.w = q.w();
	}

	void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "ENCODER");
		linear_velocity_ = msg->twist.twist.linear.x;

		RCLCPP_INFO(this->get_logger(), "a: %.2f", msg->twist.twist.linear.x);
	}

	void update_odom() {
		rclcpp::Time current_time = this->get_clock()->now();
		double deltaTime = (current_time - prev_time_).seconds();
		
		if (deltaTime <= 0.0) { // Prevents invalid/zero timesteps
			RCLCPP_WARN(this->get_logger(), "Invalid deltaTime (%.6f)! Skipping update.", deltaTime);
			return;
		}

		prev_time_ = current_time;

		// Update pose
		theta_ += angular_velocity_ * deltaTime;
		x_ += linear_velocity_ * deltaTime * cos(theta_);
  1     y_ += linear_velocity_ * deltaTime * sin(theta_);
		
		// Debug log
		RCLCPP_INFO(this->get_logger(),
            "x: %.2f, y: %.2f, theta: %.2f, linear_velocity: %.2f, angular_velocity: %.2f",
            x_, y_, theta_, linear_velocity_, angular_velocity_);

		// Publish odometry message
		auto odom_msg = nav_msgs::msg::Odometry();
		odom_msg.header.stamp = current_time; // Before: this->get_clock()->now();
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";
		odom_msg.pose.pose.position.x = x_;
		odom_msg.pose.pose.position.y = y_;
		// odom_msg.pose.pose.position.z = 0.0; // setting position.z is unnecessary since we are focusing on 2D spaces
		odom_msg.pose.pose.orientation = orientation_;
		odom_msg.pose.covariance[0] = 0.1;  // X variance
		odom_msg.pose.covariance[7] = 0.1;  // Y variance
		odom_msg.pose.covariance[35] = 0.1; // Yaw variance
		
		odom_msg.twist.twist.linear.x = linear_velocity_;
		odom_msg.twist.twist.angular.z = angular_velocity_;
		odom_publisher_->publish(odom_msg);

		// broadcast transform from odom to base_link (reuse time!)
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time; // Before: this->get_clock()->now();
		t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        // t.transform.translation.z = 0.0; // Not necessary
        t.transform.rotation = orientation_;
        tf_broadcaster_->sendTransform(t);
		
		RCLCPP_INFO(this->get_logger(), 
			"Published odometry: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
		
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

