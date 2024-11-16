#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <tf2_ros/static_transform_broadcaster.h>

class PointCloudFusionNode : public rclcpp::Node {
    private:
        // Definition of the synchronization policy
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Synchronizer;
        std::shared_ptr<Synchronizer> sync1_;
        std::shared_ptr<Synchronizer> sync2_;

        // Subscriptions for camera 1
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub1_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub1_;

        // Subscriptions for camera 2
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub2_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub2_;

        // TF2 for transformations
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Static transform broadcaster
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;

        // Point clouds for each camera
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_;

        // Publisher for the fused point cloud
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        void PointCloudCallback1(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

        
}
