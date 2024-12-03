#include "rclcpp/rclcpp.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "preprocessor.h"
#include "feature_extractor.h"
#include "feature_matcher.h"
#include "pose_estimator.h"
#include "map_manager.h"
#include "loop_closure.h"

class Ros2Integration : public rclcpp::Node {
public:
    /**
     * @brief Constructs the ROS2 integration node for SLAM.
     * Initializes subscribers for lidar and odometry data.
     */
    Ros2Integration()
        : Node("ros2_slam_integration") {
        // Subscribers
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", 10, std::bind(&Ros2Integration::lidarCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&Ros2Integration::odomCallback, this, std::placeholders::_1));
        
        // Initialize SLAM modules
        preprocessor_ = std::make_shared<Preprocessor>();
        feature_extractor_ = std::make_shared<FeatureExtractor>();
        feature_matcher_ = std::make_shared<FeatureMatcher>();
        pose_estimator_ = std::make_shared<PoseEstimator>();
        map_manager_ = std::make_shared<MapManager>();
        loop_closure_ = std::make_shared<LoopClosure>();

        // Initialize global map and poses
        global_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        current_pose_ = Eigen::Matrix4f::Identity();  // Initial pose
    }

private:
    /**
     * @brief Callback function for receiving LiDAR point cloud messages.
     * Converts the sensor_msgs/PointCloud2 message to a PCL point cloud and processes it.
     * @param cloud_msg The received PointCloud2 message containing LiDAR data.
     */
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // SLAM processing pipeline
        auto downsampled = preprocessor_->downsample(cloud, 0.1);
        auto edges = feature_extractor_->extractEdgeFeatures(downsampled);
        auto planes = feature_extractor_->extractPlaneFeatures(downsampled);
        auto transformation = feature_matcher_->matchFeatures(edges, planes);
        current_pose_ = pose_estimator_->estimatePoseWithEKF(current_pose_, transformation);
        global_map_ = map_manager_->updateMap(global_map_, downsampled, current_pose_);
    }

    /**
     * @brief Callback function for receiving odometry messages.
     * Updates the pose estimator with the incoming odometry data.
     * @param odom_msg The received Odometry message containing the robot's position and orientation.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr& odom_msg) {
        // Extract the pose from the Odometry message
        Eigen::Matrix4f odom_transform = Eigen::Matrix4f::Identity();
        odom_transform(0, 3) = odom_msg->pose.pose.position.x;
        odom_transform(1, 3) = odom_msg->pose.pose.position.y;
        odom_transform(2, 3) = odom_msg->pose.pose.position.z;

        // Update the pose based on the odometry data
        current_pose_ = pose_estimator_->estimatePoseWithEKF(current_pose_, odom_transform);
    }

    // SLAM modules
    std::shared_ptr<Preprocessor> preprocessor_;
    std::shared_ptr<FeatureExtractor> feature_extractor_;
    std::shared_ptr<FeatureMatcher> feature_matcher_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::shared_ptr<MapManager> map_manager_;
    std::shared_ptr<LoopClosure> loop_closure_;

    // SLAM state
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    Eigen::Matrix4f current_pose_;

    // ROS subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ros2Integration>());
    rclcpp::shutdown();
    return 0;
}
