#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include "opencv_final/ArmorArray.h"

class TestFixNode {
public:
    TestFixNode() : nh_("~") {
        // 订阅实际的话题
        camera_sub_ = nh_.subscribe("/hk_camera/camera_info", 1, 
                                   &TestFixNode::cameraCallback, this);
        armor_sub_ = nh_.subscribe("/ArmorDetect/armors", 10,
                                  &TestFixNode::armorCallback, this);
        
        ROS_INFO("TestFixNode initialized");
        ROS_INFO("Subscribed to /hk_camera/camera_info and /ArmorDetect/armors");
        
        // 设置发布频率（50Hz）
        publish_timer_ = nh_.createTimer(ros::Duration(0.02),  // 0.02秒 = 50Hz
                                         &TestFixNode::timerCallback, this);
    }
    
    void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        ROS_INFO_ONCE("Received camera info: fx=%.2f, fy=%.2f", 
                     msg->K[0], msg->K[4]);
    }
    
    void armorCallback(const opencv_final::ArmorArrayConstPtr& msg) {
        static int count = 0;
        count++;
        
        if (msg->armors.empty()) {
            ROS_WARN_THROTTLE(1.0, "No armors detected");
            return;
        }
        
        ROS_INFO_THROTTLE(0.5, "Frame %d: Detected %zu armors", 
                         count, msg->armors.size());
        
        // 存储最新的装甲板消息
        latest_armor_msg_ = msg;
    }
    
    void timerCallback(const ros::TimerEvent& event) {
        // 发布一个静态的基坐标系变换（世界坐标系->相机坐标系）
        geometry_msgs::TransformStamped world_to_camera;
        world_to_camera.header.stamp = ros::Time::now();
        world_to_camera.header.frame_id = "world";
        world_to_camera.child_frame_id = "camera_optical_frame";
        
        world_to_camera.transform.translation.x = 0.0;
        world_to_camera.transform.translation.y = 0.0;
        world_to_camera.transform.translation.z = 0.0;
        world_to_camera.transform.rotation.x = 0.0;
        world_to_camera.transform.rotation.y = 0.0;
        world_to_camera.transform.rotation.z = 0.0;
        world_to_camera.transform.rotation.w = 1.0;
        
        // 发布一个测试的装甲板坐标系
        geometry_msgs::TransformStamped camera_to_armor;
        camera_to_armor.header.stamp = ros::Time::now();
        camera_to_armor.header.frame_id = "camera_optical_frame";
        camera_to_armor.child_frame_id = "test_armor_0";
        
        camera_to_armor.transform.translation.x = 0.0;
        camera_to_armor.transform.translation.y = 0.0;
        camera_to_armor.transform.translation.z = 2.0;  // 在相机前方2米
        
        camera_to_armor.transform.rotation.x = 0.0;
        camera_to_armor.transform.rotation.y = 0.0;
        camera_to_armor.transform.rotation.z = 0.0;
        camera_to_armor.transform.rotation.w = 1.0;
        
        // 使用成员变量tf_broadcaster_发布TF
        tf_broadcaster_.sendTransform(world_to_camera);
        tf_broadcaster_.sendTransform(camera_to_armor);
        
        ROS_DEBUG_THROTTLE(1.0, "Published TF transforms");
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Subscriber armor_sub_;
    ros::Timer publish_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;  // 关键：成员变量，不是局部变量
    opencv_final::ArmorArrayConstPtr latest_armor_msg_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_fix_node");
    
    // 设置日志级别
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::console::notifyLoggerLevelsChanged();
    
    TestFixNode node;
    ROS_INFO("TestFixNode started");
    
    ros::spin();
    return 0;
}