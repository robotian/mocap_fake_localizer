#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class MocapFakeLocalizer : public rclcpp::Node {
public:
    Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::Point &translation, const geometry_msgs::msg::Quaternion &rotation)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
        tf2::Matrix3x3 m(q);

        for (int i = 0; i < 3; i++)
        {
        for (int j = 0; j < 3; j++)
        {
            T(i, j) = m[i][j];
        }
        }

        T(0, 3) = translation.x;
        T(1, 3) = translation.y;
        T(2, 3) = translation.z;

        return T;
    }

    Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::Vector3 &translation,
                                 const geometry_msgs::msg::Quaternion &rotation)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
        tf2::Matrix3x3 m(q);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
            T(i, j) = m[i][j];
            }
        }

        T(0, 3) = translation.x;
        T(1, 3) = translation.y;
        T(2, 3) = translation.z;

        return T;
    }

    void matrixToTransform(const Eigen::Matrix4d &matrix, geometry_msgs::msg::Point &translation, geometry_msgs::msg::Quaternion &rotation)
    {
        translation.x = matrix(0, 3);
        translation.y = matrix(1, 3);
        translation.z = matrix(2, 3);

        Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0);
        tf2::Matrix3x3 tf_m(rot(0, 0), rot(0, 1), rot(0, 2),
                            rot(1, 0), rot(1, 1), rot(1, 2),
                            rot(2, 0), rot(2, 1), rot(2, 2));

        tf2::Quaternion q;
        tf_m.getRotation(q);

        rotation.x = q.x();
        rotation.y = q.y();
        rotation.z = q.z();
        rotation.w = q.w();
    }


    MocapFakeLocalizer() : Node("mocap_fake_localizer_node") {

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // mode options:
        //   1: Use the local EKF (or odometry estimator) and a localizer (e.g., SLAM)
        //   2: Use the Mocap data for the odometry, and use a localizer
        //   3: Use the local EKF (or odometry estimator), and use the Mocap data as a localizer
        //   4: Use the Mocap data for the odometry and localization (For the Ground Truth based navigation)
        this->declare_parameter<int>("mode", 1);
        this->declare_parameter<std::vector<double>>("map_pose_wrt_ref", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}); // Pose of the map frame with respect to the reference frame (x, y, z, x, y, z, w)
        this->declare_parameter<std::string>("ref_frame", "base_mocap");
        this->declare_parameter<std::string>("ground_truth_frame", "base_link_gt");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_link_frame", "base_link");
        this->declare_parameter<std::string>("mocap_odom_topic", "odom_gt");
        this->declare_parameter<std::string>("odom_topic", "odom_filtered");

        // read parameters
        mode_ = this->get_parameter("mode").as_int();
        this->get_parameter("map_pose_wrt_ref", this->map_pose_wrt_ref_);
        // map_pose_wrt_ref_ = this->get_parameter("map_pose_wrt_ref").as_vector_double();
        ref_frame_ = this->get_parameter("ref_frame").as_string();
        gt_child_frame_ = this->get_parameter("ground_truth_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_link_frame_ = this->get_parameter("base_link_frame").as_string();
        std::string gt_topic_name = this->get_parameter("mocap_odom_topic").as_string();
        std::string odom_topic_name = this->get_parameter("odom_topic").as_string();
        

        if(mode_ < 1 || mode_ > 4) {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode parameter. Must be between 1 and 4.");
            rclcpp::shutdown();
            return;
        }

        switch(mode_) {
            case 1:
                RCLCPP_INFO(this->get_logger(), "Mode 1: Use the local EKF (or odometry estimator) and a localizer (e.g., SLAM).");
                this->ref2map_tf_broadcast();
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "Mode 2: Use the Mocap data only for the odometry, and use a localizer.");
                this->setTransMatRef2Odom();
                gt_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    gt_topic_name, 10, std::bind(&MocapFakeLocalizer::groundtruth_odom_callback, this, std::placeholders::_1));

                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "Mode 3: Use the local EKF (or odometry estimator), and use the Mocap data as a localizer.");
                this->ref2map_tf_broadcast();
                this->map2odom_tf_broadcast(true);
                // set the odom frame
                
                // odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                //     odom_topic_name, 10, std::bind(&MocapFakeLocalizer::odom_callback, this, std::placeholders::_1));
                

                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "Mode 4: Use the Mocap data for the odometry and localization (For the Ground Truth based navigation).");

                break;
        }

        // 4. Subscribe to the Mocap Odometry
        // subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     topic_name, 10, std::bind(&MocapFakeLocalizer::groundtruth_odom_callback, this, std::placeholders::_1));

        
        // odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     odom_topic_name, 10, std::bind(&MocapFakeLocalizer::odom_callback, this, std::placeholders::_1));

        // RCLCPP_INFO(this->get_logger(), "Waiting for first mocap and odom messages on: %s and %s", topic_name.c_str(), odom_topic_name.c_str());
    }

private:
    void setTransMatRef2Odom() {
        // set the homogeneous transformation matrix from the reference frame to the odom frame
        // The current ground truth 'base_footprint' frame pose relative to the 'base_mocap' frame will be the odom frame on the ref frame. 
        geometry_msgs::msg::TransformStamped ref_to_footprint_tf;
        
        try {
            ref_to_footprint_tf = tf_buffer_->lookupTransform(
                ref_frame_, gt_child_frame_, tf2::TimePointZero, tf2::Duration(std::chrono::seconds(5)));
                T_ref_to_odom_ = transformToMatrix(ref_to_footprint_tf.transform.translation, ref_to_footprint_tf.transform.rotation);
            transMat_ref2odom_set_ = true;
                
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from base_mocap to base_footprint: %s", ex.what());
        }
    }

    void map2odom_tf_broadcast(bool publist_tf = true) {
        geometry_msgs::msg::TransformStamped ref_to_footprint_tf;
        geometry_msgs::msg::TransformStamped ref_to_map_tf;
        geometry_msgs::msg::TransformStamped odom_to_baselink_tf;

        Eigen::Matrix4d T_ref2gt_tf;
        Eigen::Matrix4d T_ref2map_tf;
        Eigen::Matrix4d T_odom_to_baselink_tf;

        try {
            ref_to_footprint_tf = tf_buffer_->lookupTransform(
                ref_frame_, gt_child_frame_, tf2::TimePointZero, tf2::Duration(std::chrono::seconds(5)));
            T_ref2gt_tf = transformToMatrix(ref_to_footprint_tf.transform.translation, ref_to_footprint_tf.transform.rotation);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from base_mocap to base_footprint: %s", ex.what());
        }

        try {
            ref_to_map_tf = tf_buffer_->lookupTransform(
                ref_frame_, map_frame_, tf2::TimePointZero, tf2::Duration(std::chrono::seconds(5)));
            T_ref2map_tf = transformToMatrix(ref_to_map_tf.transform.translation, ref_to_map_tf.transform.rotation);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from base_mocap to map: %s", ex.what());
        }

        try {
            odom_to_baselink_tf = tf_buffer_->lookupTransform(
                odom_frame_, base_link_frame_, tf2::TimePointZero, tf2::Duration(std::chrono::seconds(5)));
            T_odom_to_baselink_tf = transformToMatrix(odom_to_baselink_tf.transform.translation, odom_to_baselink_tf.transform.rotation);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from odom to base_link: %s", ex.what());
        }

        Eigen::Matrix4d T_map_to_odom = T_ref2map_tf.inverse() * T_ref2gt_tf * T_odom_to_baselink_tf.inverse();

        geometry_msgs::msg::Point final_translation;
        geometry_msgs::msg::Quaternion final_rotation;
        matrixToTransform(T_map_to_odom, final_translation, final_rotation);

        geometry_msgs::msg::TransformStamped map_to_odom_tf;
        map_to_odom_tf.header.stamp = this->get_clock()->now();
        map_to_odom_tf.header.frame_id = map_frame_;
        map_to_odom_tf.child_frame_id = odom_frame_;

        map_to_odom_tf.transform.translation.x = final_translation.x;
        map_to_odom_tf.transform.translation.y = final_translation.y;
        map_to_odom_tf.transform.translation.z = final_translation.z;
        
        map_to_odom_tf.transform.rotation.w = final_rotation.w;
        map_to_odom_tf.transform.rotation.x = final_rotation.x;
        map_to_odom_tf.transform.rotation.y = final_rotation.y;
        map_to_odom_tf.transform.rotation.z = final_rotation.z;

        if(publist_tf) {
            tf_static_broadcaster_->sendTransform(map_to_odom_tf);        
            RCLCPP_INFO(this->get_logger(), "Static transform %s -> %s broadcasted.", map_frame_.c_str(), odom_frame_.c_str());
        }
    }

    void ref2map_tf_broadcast() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = ref_frame_;
        t.child_frame_id = map_frame_;         

        t.transform.translation.x = map_pose_wrt_ref_[0];
        t.transform.translation.y = map_pose_wrt_ref_[1];
        t.transform.translation.z = map_pose_wrt_ref_[2];
        t.transform.rotation.x = map_pose_wrt_ref_[3];
        t.transform.rotation.y = map_pose_wrt_ref_[4];
        t.transform.rotation.z = map_pose_wrt_ref_[5];
        t.transform.rotation.w = map_pose_wrt_ref_[6];
        
        tf_static_broadcaster_->sendTransform(t);
        
        RCLCPP_INFO(this->get_logger(), "Static transform %s -> map broadcasted.", ref_frame_.c_str());
    }

    void groundtruth_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if(!transMat_ref2odom_set_){
            RCLCPP_WARN(this->get_logger(), "Reference to odom transform not set yet. Ignoring ground truth odometry data.");
            this->setTransMatRef2Odom();
            return;
        }

        // publish odom->base_link transform based on the ground truth pose and the reference to odom transform
        Eigen::Matrix4d T_ref_to_gt = transformToMatrix(msg->pose.pose.position,  msg->pose.pose.orientation);
        Eigen::Matrix4d T_odom_to_baselink = T_ref_to_odom_.inverse() * T_ref_to_gt;
        geometry_msgs::msg::Point final_translation;
        geometry_msgs::msg::Quaternion final_rotation;
        matrixToTransform(T_odom_to_baselink, final_translation, final_rotation);

        geometry_msgs::msg::TransformStamped odom_to_baselink_tf;

        odom_to_baselink_tf.header.stamp = this->get_clock()->now();
        odom_to_baselink_tf.header.frame_id = odom_frame_;
        odom_to_baselink_tf.child_frame_id = base_link_frame_;

        odom_to_baselink_tf.transform.translation.x = final_translation.x;
        odom_to_baselink_tf.transform.translation.y = final_translation.y;
        odom_to_baselink_tf.transform.translation.z = final_translation.z;
        
        odom_to_baselink_tf.transform.rotation.w = final_rotation.w;
        odom_to_baselink_tf.transform.rotation.x = final_rotation.x;
        odom_to_baselink_tf.transform.rotation.y = final_rotation.y;
        odom_to_baselink_tf.transform.rotation.z = final_rotation.z;

        tf_broadcaster_->sendTransform(odom_to_baselink_tf);



        // publish ref->map transform based on the ground truth pose and the reference to odom transform
        // first, get the map-to-odom transform
        try {
            geometry_msgs::msg::TransformStamped map_to_odom_tf = tf_buffer_->lookupTransform(
                map_frame_, odom_frame_, tf2::TimePointZero, tf2::Duration(std::chrono::seconds(5)));
            Eigen::Matrix4d T_map_to_odom = transformToMatrix(map_to_odom_tf.transform.translation, map_to_odom_tf.transform.rotation);
            Eigen::Matrix4d T_ref_to_map = T_ref_to_gt * T_odom_to_baselink.inverse() * T_map_to_odom.inverse();
            geometry_msgs::msg::Point map_translation;
            geometry_msgs::msg::Quaternion map_rotation;
            matrixToTransform(T_ref_to_map, map_translation, map_rotation);

            geometry_msgs::msg::TransformStamped ref_to_map_tf;

            ref_to_map_tf.header.stamp = this->get_clock()->now();
            ref_to_map_tf.header.frame_id = ref_frame_;
            ref_to_map_tf.child_frame_id = map_frame_;
            ref_to_map_tf.transform.translation.x = map_translation.x;
            ref_to_map_tf.transform.translation.y = map_translation.y;
            ref_to_map_tf.transform.translation.z = map_translation.z;
            ref_to_map_tf.transform.rotation.w = map_rotation.w;
            ref_to_map_tf.transform.rotation.x = map_rotation.x;
            ref_to_map_tf.transform.rotation.y = map_rotation.y;
            ref_to_map_tf.transform.rotation.z = map_rotation.z;

            tf_broadcaster_->sendTransform(ref_to_map_tf);
            RCLCPP_INFO(this->get_logger(), "Static transform %s -> %s broadcasted.", ref_frame_.c_str(), map_frame_.c_str());
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from map to odom: %s", ex.what());
        }
    }

    // void groundtruth_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    //     // set the initial pose of the robot in the MoCap as the map frame 
    //     if (initialized_) return;

    //     RCLCPP_INFO(this->get_logger(), "Received first Mocap data. Establishing %s -> %s transform.", ref_frame_.c_str(), child_frame_.c_str());

    //     geometry_msgs::msg::TransformStamped t;

    //     t.header.stamp = this->get_clock()->now();
    //     t.header.frame_id = ref_frame_;
    //     t.child_frame_id = child_frame_;         

    //     t.transform.translation.x = msg->pose.pose.position.x;
    //     t.transform.translation.y = msg->pose.pose.position.y;
    //     t.transform.translation.z = msg->pose.pose.position.z;
    //     t.transform.rotation.w = msg->pose.pose.orientation.w;
    //     t.transform.rotation.x = msg->pose.pose.orientation.x;
    //     t.transform.rotation.y = msg->pose.pose.orientation.y;
    //     t.transform.rotation.z = msg->pose.pose.orientation.z;
        
    //     ref_to_child_transform_ = t.transform;

    //     tf_static_broadcaster_->sendTransform(t);
        
    //     initialized_ = true;
    //     RCLCPP_INFO(this->get_logger(), "Static transform %s -> %s broadcasted.", t.header.frame_id.c_str(), t.child_frame_id.c_str());
        
    //     // We can stop the subscription now to save resources
    //     gt_odom_subscription_.reset();
    // }



    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!initialized_) {
            RCLCPP_WARN(this->get_logger(), "Map frame not initialized yet. Ignoring odometry data.");
            return;
        }


        // transform of the odometry data from the odom topic 
        geometry_msgs::msg::Transform odom_baselink_transform;
        odom_baselink_transform.translation.x = msg->pose.pose.position.x;
        odom_baselink_transform.translation.y = msg->pose.pose.position.y;
        odom_baselink_transform.translation.z = msg->pose.pose.position.z;
        odom_baselink_transform.rotation.w = msg->pose.pose.orientation.w;
        odom_baselink_transform.rotation.x = msg->pose.pose.orientation.x;
        odom_baselink_transform.rotation.y = msg->pose.pose.orientation.y;
        odom_baselink_transform.rotation.z = msg->pose.pose.orientation.z;

        // Get transform from 'map' frame to 'odom' frame
        try {
            geometry_msgs::msg::TransformStamped mocap_to_footprint_tf = tf_buffer_->lookupTransform(
                "base_mocap", "base_footprint", tf2::TimePointZero);

            T_mocap_to_footprint_ = transformToMatrix(mocap_to_footprint_tf.transform.translation, mocap_to_footprint_tf.transform.rotation);

            geometry_msgs::msg::TransformStamped mocap_to_map_tf = tf_buffer_->lookupTransform(
                "base_mocap", "map", tf2::TimePointZero);

            T_mocap_to_map_ = transformToMatrix(mocap_to_map_tf.transform.translation, mocap_to_map_tf.transform.rotation);

            geometry_msgs::msg::TransformStamped odom_to_baselink_tf = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);

            T_odom_to_baselink_ = transformToMatrix(odom_to_baselink_tf.transform.translation, odom_to_baselink_tf.transform.rotation);
            

            Eigen::Matrix4d T_map_to_odom = T_mocap_to_map_.inverse() * T_mocap_to_footprint_ * T_odom_to_baselink_;
            geometry_msgs::msg::Point final_translation;
            geometry_msgs::msg::Quaternion final_rotation;
            matrixToTransform(T_map_to_odom, final_translation, final_rotation);


            geometry_msgs::msg::TransformStamped map_to_odom_tf;

            map_to_odom_tf.header.stamp = this->get_clock()->now();
            map_to_odom_tf.header.frame_id = "map";
            map_to_odom_tf.child_frame_id = "odom";

            map_to_odom_tf.transform.translation.x = final_translation.x;
            map_to_odom_tf.transform.translation.y = final_translation.y;
            map_to_odom_tf.transform.translation.z = final_translation.z;
            
            map_to_odom_tf.transform.rotation.w = final_rotation.w;
            map_to_odom_tf.transform.rotation.x = final_rotation.x;
            map_to_odom_tf.transform.rotation.y = final_rotation.y;
            map_to_odom_tf.transform.rotation.z = final_rotation.z;
            
            tf_static_broadcaster_->sendTransform(map_to_odom_tf);


        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from base_mocap to base_footprint: %s", ex.what());
        }



        // We can add any additional logic here if needed, but for now we just log the reception of odometry data.
        RCLCPP_INFO(this->get_logger(), "Received odometry data on %s.", msg->header.frame_id.c_str());
    }

    bool initialized_ = false;
    bool transMat_ref2odom_set_ = false;
    std::string ref_frame_;
    // std::string child_frame_;
    std::string gt_child_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_link_frame_;
    int mode_;
    std::vector<double> map_pose_wrt_ref_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;    
    geometry_msgs::msg::Transform ref_to_child_transform_;

    Eigen::Matrix4d T_mocap_to_footprint_; 
    Eigen::Matrix4d T_mocap_to_map_; 
    Eigen::Matrix4d T_odom_to_baselink_; 
    Eigen::Matrix4d T_ref_to_odom_; 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapFakeLocalizer>());
    rclcpp::shutdown();
    return 0;
}