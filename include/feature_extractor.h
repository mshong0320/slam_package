#ifndef FEATURE_EXTRACTOR_H 
#define FEATURE_EXTRACTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @class FeatureExtractor
 * @brief A class for extracting features from LIDAR point clouds using different 3D feature detection algorithms.
 * This includes edge feature extraction, plane feature extraction, and keypoint detection using CUDA-accelerated computations.
 */
class FeatureExtractor {
public:
    /**
     * @brief Extracts edge features from the point cloud using curvature-based methods accelerated by CUDA.
     * The curvature of each point is computed in parallel on the GPU, and points with high curvature are considered edge features.
     * 
     * @param cloud The input point cloud.
     * @return A point cloud containing the extracted edge features (points with high curvature).
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractEdgeFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief Extracts plane features from the point cloud. Currently implemented as a simplified version of edge feature extraction.
     * The method uses a similar approach to `extractEdgeFeatures` with adjusted thresholds for detecting plane features.
     * 
     * @param cloud The input point cloud.
     * @return A point cloud containing the extracted plane features (currently using edge feature extraction logic).
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractPlaneFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief Extracts keypoints from the point cloud using a 3D keypoint detection method.
     * This is a placeholder function that can be implemented with specific algorithms for keypoint detection.
     * 
     * @param cloud The input point cloud.
     * @return A point cloud containing the extracted keypoints.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractKeypoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

#endif
