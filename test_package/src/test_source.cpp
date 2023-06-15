#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cstring>
//do quaternionow
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class TestNode : public rclcpp::Node{
    public:
        TestNode(): Node("test_node"){
            subscriberTF = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf",10,std::bind(&TestNode::tfCallback, this, std::placeholders::_1));
            subscriberOdometry = this->create_subscription<nav_msgs::msg::Odometry>("/run_slam/camera_pose", 10, std::bind(&TestNode::odomCallback, this, std::placeholders::_1));
        }
        private:
        void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const{
             for (const auto& transform : msg->transforms) {
            
             if(strcmp(msg->transforms[0].header.frame_id.c_str(), "map") == 0 &&
    			strcmp(msg->transforms[0].child_frame_id.c_str(), "odom") == 0){
    			 RCLCPP_INFO(this->get_logger(),"Odebrana wiadomosc typu tf:");
                        RCLCPP_INFO(get_logger(), "Transform from '%s' to '%s':", transform.child_frame_id.c_str(), transform.header.frame_id.c_str());
      			RCLCPP_INFO(this->get_logger(), "  Translation: x=%.2f, y=%.2f, z=%.2f",
                  transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z);

                    tf2::Quaternion tf_quat(transform.transform.rotation.x,
                  transform.transform.rotation.y,
                  transform.transform.rotation.z,
                  transform.transform.rotation.w);
                    tf2::Matrix3x3 tf_mat(tf_quat);
                    double roll, pitch, yaw;
                    tf_mat.getRPY(roll, pitch, yaw);

      RCLCPP_INFO(this->get_logger(), "  Rotation: x=%.2f, y=%.2f, z=%.2f",
                  roll, pitch, yaw);
            }

        }
        }
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // Przelicz quaternion na kąty Eulera (roll, pitch, yaw)
    tf2::Quaternion tf_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 tf_mat(tf_quat);
    double roll, pitch, yaw;
    tf_mat.getRPY(roll, pitch, yaw);

    // Wyświetlanie pozycji, rotacji i kąty Eulera w terminalu
    RCLCPP_INFO(this->get_logger(), "Odebrano wiadomość typu Odometry:\n"
                                    "Pozycja: x=%.2f, y=%.2f, z=%.2f\n"
                                    "Rotacja (quaternion): x=%.2f, y=%.2f, z=%.2f, w=%.2f\n"
                                    "Kąty Eulera (roll, pitch, yaw): roll=%.2f, pitch=%.2f, yaw=%.2f",
                x, y, z, qx, qy, qz, qw, roll, pitch, yaw);
        }
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriberTF;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriberOdometry;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
