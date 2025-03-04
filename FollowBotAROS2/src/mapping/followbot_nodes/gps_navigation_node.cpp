#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

/** // TODO: set-up action client for dynamic navigation in case of failure
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>
*/

class GPSNavNode : public rclcpp::Node {
 public:
	 GPSNavNode() : Node("gps_navigation_node") {
		 gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
			"gps/fix", 10, 
			std::bind(&GPSNavNode::gps_callback, this, std::placeholders::1));

		 goal_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
			"/set_goal", 10
			std::bind(&GPSNavNode::goal_callback, this, std::placeholders::1));

		 goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
			"/goal_pose", 1);

		 tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		/**
		action_client_ = rclcpp_action::create_client<NavigateToPose>(
			this, "navigate_to_pose");

		while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) 
			RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
	
		RCLCPP_INFO(this->get_logger(), "Nav2 action server available.");
		*/

		 RCLCPP_INFO(this->get_logger(), "GPS Navigation Node initialized.");
	 }

 private:
	 void gps_callback(const sensor_msg::msg::NavSatFix::SharedPtr msg) {
		 int zone;    // zero means UPS coordinate system vs. UTM
		 bool northp; // false means south, true means north
		 
		 GeographicLib::UTMUPS::Forward(
			msg->latitude, msg->longitude, zone, northp, current_x_, current_y_);

		 // publish transform (robot's real-time position in Nav2's map)
		 geometry_msgs::msg::Transformstamped t;
		 t.header.stamp = this->getclock()->now();
		 t.header.frame_id = "map";
		 t.child_frame_id = "base_link";
		 t.transform.translation.x = current_x_;
		 t.transform.translation.y = current_y_;
		 t.transform.translation.z = 0.0;
		 t.transform.rotation.w = 1.0;

		 tf_broadcaster_->sendTransform(transform);

		 RCLCPP_IFNO(this->get_logger(), 
			"Updated Current Position: X = %.2f, Y = .%2f", current_x_, current_y_);
	 }

	 void goal_callback() {
		double goal_x, goal_y;
		int zone;
		bool northp;

		GeographicLib::UTMUPS::Forward(
			msg->latitude, msg->longitude, zone, northp, goal_x, goal_y);

		geometry_msgs::msg::PoseStamped goal_msg;
		goal_msg.header.stamp = this->get_clock()->now();
		goal_msg.header.frame_id = "map";
		goal_msg.pose.position.x = goal_x;
		goal_msg.pose.position.y = goal_y;
		goal_msg.pose.position.z = 0.0;
		goal_msg.pose.orientation.w = 1.0;

		goal_pub_->publish(goal_msg);
		RCLCPP_INFO(this->get_logger(), 
			"Goal Set: X = %.2f, Y = %.2f", goal_x, goal_y);
	 }

	 rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
	 rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_sub_;
	 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
	 std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	 double current_x_, current_y_;
	 
	 // rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GPSNavNode>());
	rclcpp::shutdown();
	return 0;
}
