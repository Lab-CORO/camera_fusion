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


class PointCloudFusionNode : public rclcpp::Node {
public:
    PointCloudFusionNode() : Node("pointcloud_fusion_node")
    {
        // Initialiser les abonnements pour la caméra 1
        depth_image_sub1_.subscribe(this, "/depth/image_raw");
        camera_info_sub1_.subscribe(this, "/depth/camera_info");
        sync1_.reset(new Synchronizer(MySyncPolicy(10), depth_image_sub1_, camera_info_sub1_));
        sync1_->registerCallback(std::bind(&PointCloudFusionNode::imageCallback1, this, std::placeholders::_1, std::placeholders::_2));

        // Initialiser les abonnements pour la caméra 2
        depth_image_sub2_.subscribe(this, "/camera/camera/depth/image_rect_raw");
        camera_info_sub2_.subscribe(this, "/camera/camera/color/camera_info");
        sync2_.reset(new Synchronizer(MySyncPolicy(10), depth_image_sub2_, camera_info_sub2_));
        sync2_->registerCallback(std::bind(&PointCloudFusionNode::imageCallback2, this, std::placeholders::_1, std::placeholders::_2));

        // Initialiser le publisher pour le nuage de points fusionné
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("fused_pointcloud", 10);
    } // coucou

        // Initialiser TF2 pour les transformations
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    // Définition de la politique de synchronisation
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Synchronizer;
    std::shared_ptr<Synchronizer> sync1_;
    std::shared_ptr<Synchronizer> sync2_;

    // Abonnements pour la caméra 1
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub1_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub1_;

    // Abonnements pour la caméra 2
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub2_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub2_;

    // TF2 pour les transformations
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Nuages de points pour chaque caméra
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_;

    // Publisher pour le nuage de points fusionné
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    // Fonctions de rappel des 2 caméras 
    void imageCallback1(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        cloud1_ = convertDepthImageToPointCloud(image_msg, info_msg);
        fuseClouds();
    }

    void imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        cloud2_ = convertDepthImageToPointCloud(image_msg, info_msg);
        fuseClouds();
    }

    // Fonction pour convertir une image de profondeur en nuage de points 
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthImageToPointCloud(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                                                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        // Déterminer le facteur d'échelle
        float depth_scale = 1.0f;
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
            depth_scale = 0.001f; // Convertir de millimètres en mètres
        } 
        else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) 
        {
            depth_scale = 1.0f;
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Encodage d'image de profondeur non supporté : %s", depth_msg->encoding.c_str());
            return nullptr;
        }

        // Convertir l'image de profondeur en image OpenCV pour faciliter la conversion en pointcloud
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);

        // Obtenir les paramètres intrinsèques de la caméra
        double fx = info_msg->k[0];
        double fy = info_msg->k[4];
        double cx = info_msg->k[2];
        double cy = info_msg->k[5];

        // Créer le nuage de points dans le cadre de la caméra
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // Parcourir chaque pixel de l'image de profondeur pour la conversion en pointcloud
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

        // Transformer le nuage de points dans le cadre 'map'
        std::string from_frame = depth_msg->header.frame_id;
        std::string to_frame = "map";
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Impossible de transformer %s vers %s: %s", from_frame.c_str(), to_frame.c_str(), ex.what());
            return nullptr;
        }

        Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped.transform);

        // Appliquer la transformation au nuage de points
        pcl::transformPointCloud(*cloud, *cloud, transform);

        return cloud;
    }

    // Fonction pour fusionner les nuages de points
    void fuseClouds()
    {
        // Vérifier si les deux nuages de points sont disponibles si ils sont vides alors la fusion ne se fera pas
        if (cloud1_ && !cloud1_->empty() && cloud2_ && !cloud2_->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr fused_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            *fused_cloud = *cloud1_ + *cloud2_;
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*fused_cloud, output);
            output.header.frame_id = "map"; //depth_camera_link
            pub_->publish(output);

            // Réinitialiser les nuages de points après publication
            cloud1_.reset();
            cloud2_.reset();
        }
    }
};


int main(int argc, char **argv)
{
    // Initialiser ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

