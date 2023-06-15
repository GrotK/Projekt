/*#ifndef TEST_NODE_HPP_
#define TEST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
namespace testnode
{
class TestNode : public rclcpp::Node
{
    public:
        TestNode();
    private:
        void topic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber;
};
}
#endif */