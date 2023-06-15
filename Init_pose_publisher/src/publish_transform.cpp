#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("publish_transform");

  auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = argv[1];
  transformStamped.child_frame_id = argv[2];
  transformStamped.transform.translation.x = atof(argv[3]);
  transformStamped.transform.translation.y = atof(argv[4]);
  transformStamped.transform.translation.z = atof(argv[5]);

  tf2::Quaternion q;
  q.setRPY(atof(argv[6]), atof(argv[7]), atof(argv[8]));
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  rclcpp::Rate rate(10.0);
  while (rclcpp::ok())
  {
    transformStamped.header.stamp = node->now();
    broadcaster->sendTransform(transformStamped);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
