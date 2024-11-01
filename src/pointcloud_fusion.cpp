#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudFusionNode : public rclcpp::Node {
public:
    PointCloudFusionNode() : Node("pointcloud_fusion_node") 
    {
        //s'abonne au topic conetant le pointcloud de la RealSense et Azaure Kinet
        // et crée le topic qui vas contenri le pointcloud fusionner
        sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "topic_image_realsense1", 10, std::bind(&PointCloudFusionNode::callback1, this, std::placeholders::_1));
        sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "topic_image_Azure kient", 10, std::bind(&PointCloudFusionNode::callback2, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("fused_pointcloud", 10);
    } // coucou

private:

    // pour manipulation des pointclouds
    void callback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud1);
        cloud1_ = cloud1;
        fuseClouds();
    }

    void callback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud2);
        cloud2_ = cloud2;
        fuseClouds();
    }

    void fuseClouds() 
    {
        //verifie si le cloud contient des données et fusionne si oui 
        if (cloud1_ && !cloud1_->empty() && cloud2_ && !cloud2_->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr fused_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            *fused_cloud = *cloud1_ + *cloud2_;
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*fused_cloud, output);
            output.header.frame_id = "map";
            pub_->publish(output);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_;
};

int main(int argc, char **argv) 
{
    //démarre ros2
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFusionNode>());
    rclcpp::shutdown();
    return 0;
}
