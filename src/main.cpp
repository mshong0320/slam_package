#include <filesystem> 
#include <pcl/io/pcd_io.h>
#include "preprocessor.h"
#include "feature_extractor.h"
#include "feature_matcher.h"
#include "pose_estimator.h"
#include "map_manager.h"
#include "loop_closure.h"

namespace fs = std::filesystem;

/**
 * @brief Main function for processing LiDAR data and performing SLAM.
 * 
 * This program processes a series of LiDAR point cloud data (PCD files) using a SLAM pipeline,
 * which includes preprocessing, feature extraction, feature matching, pose estimation, and map updating.
 * The final output is a global map created from the input point clouds, which is saved as a PCD file.
 * 
 * The pipeline follows these steps:
 * 1. Downsample point clouds to reduce size.
 * 2. Extract edge and plane features.
 * 3. Match features across frames.
 * 4. Estimate pose using an Extended Kalman Filter (EKF).
 * 5. Update the global map with the current frame's data and pose.
 * 6. Optionally, detect and optimize loop closure to improve map accuracy.
 * 
 * The processed map is saved as a PCD file for further use.
 * 
 * @return 0 on success.
 */
int main() {
    // SLAM modules
    Preprocessor preprocessor; ///< Preprocessing module for point cloud data
    FeatureExtractor feature_extractor; ///< Feature extraction module to identify keypoints
    FeatureMatcher feature_matcher; ///< Feature matching module to match features between frames
    PoseEstimator pose_estimator; ///< Pose estimation module using an EKF approach
    MapManager map_manager; ///< Map management module to maintain and update the global map
    LoopClosure loop_closure; ///< Loop closure detection and optimization module

    // Map and poses
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>()); ///< Global map that stores all processed data
    std::vector<Eigen::Matrix4f> poses; ///< Vector to store the poses of each frame

    // Directory of PCD files (adjust to your dataset location)
    std::string dataset_dir = "/path/to/your/pcd_files/"; ///< Path to the directory containing PCD files
    auto lidar_files = fs::directory_iterator(dataset_dir); ///< Iterator to read all PCD files in the directory

    Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity(); ///< Initial pose (identity matrix)
    poses.push_back(current_pose); ///< Add initial pose to the poses vector

    // Process each LiDAR frame
    for (const auto& file : lidar_files) {
        if (file.path().extension() == ".pcd") { // Only process PCD files
            // Load point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(file.path().string(), *cloud) == -1) {
                PCL_ERROR("Could not read file.\n");
                continue; // Skip the current file if it fails to load
            }

            // Process pipeline
            auto downsampled = preprocessor.downsample(cloud, 0.1); ///< Downsample the point cloud
            auto edges = feature_extractor.extractEdgeFeatures(downsampled); ///< Extract edge features
            auto planes = feature_extractor.extractPlaneFeatures(downsampled); ///< Extract plane features
            auto transformation = feature_matcher.matchFeatures(edges, planes); ///< Match features between frames
            current_pose = pose_estimator.estimatePoseWithEKF(current_pose, transformation); ///< Estimate the current pose
            global_map = map_manager.updateMap(global_map, downsampled, current_pose); ///< Update the global map
            poses.push_back(current_pose); ///< Store the current pose

            // Display progress
            std::cout << "Processed file: " << file.path().filename() << std::endl;
        }
    }

    // Loop closure (optional)
    auto candidates = loop_closure.detectLoopClosure(poses, current_pose); ///< Detect loop closures in the poses
    loop_closure.optimizePoseGraph(poses); ///< Optimize the pose graph to reduce drift

    // Save final map
    pcl::io::savePCDFileBinary("global_map.pcd", *global_map); ///< Save the global map as a PCD file
    std::cout << "Map saved as global_map.pcd" << std::endl;

    return 0; ///< Return 0 indicating successful completion
}
