#include "pointcloud_fusion.hpp"
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

/**
     * Here we create the node responsible for merging the point clouds from two cameras.
     * We have callback functions that handle the received point clouds.
     * Since the camera topics provide basic 2D images, we also have a function
     * that converts the 2D images into point clouds before merging them.
*/
class PointCloudFusionNode : public rclcpp::Node {
public:
    
    PointCloudFusionNode(string depth_image_sub1_topic,string camera_info_sub1_topic,string depth_image_sub2_topic, string camera_info_sub2_topic ) : Node("pointcloud_fusion_node"),
                             static_broadcaster_(this)  // Initialize the StaticTransformBroadcaster
    {
        // Initialize subscriptions for camera 1
        depth_image_sub1_.subscribe(this, depth_image_sub1_topic);
        camera_info_sub1_.subscribe(this, camera_info_sub1_topic);
        sync1_.reset(new Synchronizer(MySyncPolicy(10), depth_image_sub1_, camera_info_sub1_));
        sync1_->registerCallback(std::bind(&PointCloudFusionNode::PointCloudCallback1, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize subscriptions for camera 2
        depth_image_sub2_.subscribe(this, depth_image_sub2_topic);
        camera_info_sub2_.subscribe(this, camera_info_sub2_topic );
        sync2_.reset(new Synchronizer(MySyncPolicy(10), depth_image_sub2_, camera_info_sub2_));
        sync2_->registerCallback(std::bind(&PointCloudFusionNode::PointCloudCallback2, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize the publisher for the fused point cloud
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("fused_pointcloud", 10);

        // Initialize TF2 for transformations
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publish static transformations
        publish_static_transforms();
    }

private:
    
    // Callback functions for the two cameras that retrieve the point clouds after their conversion
    void PointCloudCallback1(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        cloud1_ = convertDepthImageToPointCloud(image_msg, info_msg);
        fuseClouds();
    }

    void PointCloudCallback2(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        cloud2_ = convertDepthImageToPointCloud(image_msg, info_msg);
        fuseClouds();
    }

    // Function to convert a depth image into a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthImageToPointCloud(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                                                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        // Determine the scale factor
        float depth_scale = 1.0f;
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
            depth_scale = 0.001f; // Convert from millimeters to meters
        } 
        else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) 
        {
            depth_scale = 1.0f;
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported depth image encoding: %s", depth_msg->encoding.c_str());
            return nullptr;
        }

        // Convert the depth image to an OpenCV image for easier point cloud conversion
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);

        // Get the camera intrinsic parameters
        double fx = info_msg->k[0];
        double fy = info_msg->k[4];
        double cx = info_msg->k[2];
        double cy = info_msg->k[5];

        // Create the point cloud in the camera frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // Iterate over each pixel in the depth image to convert to point cloud
        for (int v = 0; v < cv_ptr->image.rows; ++v) {
            for (int u = 0; u < cv_ptr->image.cols; ++u) 
            {
                float z = 0.0f;
                if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) 
                {
                    uint16_t depth = cv_ptr->image.at<uint16_t>(v, u);
                    z = depth * depth_scale;
                } 
                else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) 
                {
                    z = cv_ptr->image.at<float>(v, u) * depth_scale;
                }

                if (z > 0.0f && std::isfinite(z)) 
                {
                    float x = (u - cx) * z / fx;
                    float y = (v - cy) * z / fy;

                    pcl::PointXYZ point;
                    point.x = x;
                    point.y = y;
                    point.z = z;

                    cloud->points.push_back(point);
                }
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        // Transform the point cloud into the 'map' frame
        std::string from_frame = depth_msg->header.frame_id;
        std::string to_frame = "map";
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", from_frame.c_str(), to_frame.c_str(), ex.what());
            return nullptr;
        }

        Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped.transform);

        // Apply the transformation to the point cloud
        pcl::transformPointCloud(*cloud, *cloud, transform);

        return cloud;
    }

    // Function to fuse the point clouds
    void fuseClouds()
    {
        // Check if both point clouds are available; if they are empty, the fusion will not occur
        if (cloud1_ && !cloud1_->empty() && cloud2_ && !cloud2_->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr fused_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            *fused_cloud = *cloud1_ + *cloud2_;
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*fused_cloud, output);
            output.header.frame_id = "map";
            pub_->publish(output);

            // Reset the point clouds after publishing
            cloud1_.reset();
            cloud2_.reset();
        }
    }

    // Function to publish static transformations
    void publish_static_transforms()
    {
        // Static transformation from 'map' to 'camera_depth_optical_frame'
        geometry_msgs::msg::TransformStamped static_transform_cam1;
        static_transform_cam1.header.stamp = this->get_clock()->now();
        static_transform_cam1.header.frame_id = "map"; //base_frame
        static_transform_cam1.child_frame_id = "camera_depth_optical_frame";

        static_transform_cam1.transform.translation.x = 0.0;
        static_transform_cam1.transform.translation.y = 0.0;
        static_transform_cam1.transform.translation.z = 0.0;
        static_transform_cam1.transform.rotation.x = 0.0;
        static_transform_cam1.transform.rotation.y = 0.0;
        static_transform_cam1.transform.rotation.z = 0.0;
        static_transform_cam1.transform.rotation.w = 1.0;

        // Static transformation from 'map' to 'depth_camera_link'
        geometry_msgs::msg::TransformStamped static_transform_cam2;
        static_transform_cam2.header.stamp = this->get_clock()->now();
        static_transform_cam2.header.frame_id = "map";
        static_transform_cam2.child_frame_id = "depth_camera_link";

        static_transform_cam2.transform.translation.x = 0.0;
        static_transform_cam2.transform.translation.y = 0.0;
        static_transform_cam2.transform.translation.z = 0.0;
        static_transform_cam2.transform.rotation.x = 0.0;
        static_transform_cam2.transform.rotation.y = 0.0;
        static_transform_cam2.transform.rotation.z = 0.0;
        static_transform_cam2.transform.rotation.w = 1.0;

        // Publish the static transformations
        static_broadcaster_.sendTransform(static_transform_cam1);
        static_broadcaster_.sendTransform(static_transform_cam2);
    }
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    string depth_image_sub1_topic = argv[0];
    string camera_info_sub1_topic = argv[1];
    string depth_image_sub1_topic = argv[2];
    string camera_info_sub2_topic = argv[3];
    auto node = std::make_shared<PointCloudFusionNode>(depth_image_sub1_topic);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
