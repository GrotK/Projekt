#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class InitPosePublisher : public rclcpp::Node{
    public:
        InitPosePublisher(): Node("Init_pose_node"){//konstruktor przykadowy
            RCLCPP_INFO(this->get_logger(),"Init pose node inicjalizacja");//zwykly print

            publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&InitPosePublisher::timer_callback, this));
        }
        private:
    void timer_callback()
    {
      auto message = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

        message->header.stamp = this->now();
        message->header.frame_id = "init_frame";
        message->pose.pose.position.x = 1.0;
        message->pose.pose.position.y = 2.0;
        message->pose.pose.position.z = 3.0;

            message->pose.pose.orientation.x = 0.0;
    message->pose.pose.orientation.y = 0.0;
    message->pose.pose.orientation.z = 0.0;
    message->pose.pose.orientation.w = 1.0;

    message->pose.covariance[0] = 0.1;
    message->pose.covariance[7] = 0.1;
    message->pose.covariance[14] = 0.1;


      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message->header.frame_id.c_str());

      publisher_->publish(*message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);//ros2 comunication
    auto node = std::make_shared<InitPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
