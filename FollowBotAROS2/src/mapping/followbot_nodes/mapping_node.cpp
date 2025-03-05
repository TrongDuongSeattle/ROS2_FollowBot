#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <algorithm> // for data validation

class LidarPreprocessor : public rclcpp::Node {
 public:
	 LidarPreprocessor() : Node("lidar_preprocessor") {
		 /**
		 map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
		 */
		 
		 lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"scan", 10, 
			std::bind(&LidarPreprocessor::lidar_callback, this, std::placeholders::_1)
		 );

		 tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
	 }

 private:
	 void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		 // validate LIDAR data and filter out NaNs or infinite values
		 auto valid_ranges = msg->ranges;
		 valid_ranges.erase(
			std::remove_if(valid_ranges.begin(), valid_ranges.end(), [](float range) {
				return !std::isfinite(range) || range <= 0.0f;
			}),
			valid_ranges.end()
		 );

		 if (valid_ranges.empty()) {
			RCLCPP_WARN(this->get_logger(), "All LIDAR data is invalid in this scan.");
			return;
		 }		 
		 
		 // testing out fields of LaserScan
		 for (size_t i = 0; i < msg->ranges.size(); ++i) {
			float angle = msg->angle_min + i * msg->angle_increment;
			float distance = msg->ranges[i];

			// filter out invalid ranges
			if (distance >= msg->range_min && distance <= msg->range_max) {
				RCLCPP_INFO(this->get_logger(), 
					"Angle: %.2f radians, Distance: %.2f meters", angle, distance);
			}
				
		 }

		 /**
		 auto map_msg = nav_msgs::msg::OccupancyGrid();
		 map_msg.header.stamp = this->get_clock()->now();
		 map_msg.header.frame_id = "map";
		 map_msg.info.resolution = 0.05; // 5cm per cell
		 map_msg.info.height = 100;
		 map_msg.info.width = 100;  // 100 cells
		 map_msg.info.origin.position.x = - 2.5;
		 map_msg.info.origin.position.y = - 2.5; // 100 * 0.05 = 5m -> (2.5, 2.5) is center
		 map_msg.data.resize(map_msg.info.width * map_msg.info.height, -1); // uknown space

		 for (size_t i = 0; i < valid_ranges.size(); ++i) {
			int cell index = i % (map_msg.info.width * map_msg.info.height);
			map_msg.data[cell_index] = 0; // mark as free space
		 }

		 map_publisher_->publish(map_msg);
		 */

		 // broadcast a static transform to help `slam_toolbox` associate incoming data
		 geometry_msgs::msg::TransformStamped t;
		 t.header.stamp = this->get_clock()->now();
		 t.header.frame_id = "map";
		 t.child_frame_id = "base_link";
		 t.transform.translation.x = 0.0;
		 t.transform.translation.y = 0.0;
		 t.transform.translation.z = 0.0;
		 t.transform.rotation.w = 1.0;   // No rotation
		 
		 tf_broadcaster_->sendTransform(t);
	 }

	 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
	 // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
	 std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarPreprocessor>());
	rclcpp::shutdown();
	return 0;
}
