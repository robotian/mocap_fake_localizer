#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class MocapFakeEkf : public rclcpp::Node {
public:
    MocapFakeEkf() : Node("mocap_fake_ekf_node") {
        // Parameters
        this->declare_parameter<std::string>("mocap_odom_topic", "ground_truth/odom");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_link_frame", "base_link");

        std::string input_topic = this->get_parameter("mocap_odom_topic").as_string();

        // Publishers & Broadcasters
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_filtered", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscription
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic, 10, std::bind(&MocapFakeEkf::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Fake EKF Node started. Subscribed to: %s", input_topic.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::string odom_frame = this->get_parameter("odom_frame").as_string();
        std::string base_frame = this->get_parameter("base_link_frame").as_string();

        // 1. Publish the Odometry message
        auto filtered_odom = *msg;
        filtered_odom.header.frame_id = odom_frame;
        filtered_odom.child_frame_id = base_frame;
        odom_pub_->publish(filtered_odom);

        // 2. Broadcast the TF (odom -> base_link)
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = odom_frame;
        t.child_frame_id = base_frame;

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapFakeEkf>());
    rclcpp::shutdown();
    return 0;
}