#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

/**
 * @class FeatureMatcher
 * @brief Matches 3D features from point clouds to identify correspondences.
 */
class FeatureMatcher {
public:
    /**
     * @brief Matches features between two point clouds based on their keypoints.
     * @param cloud1 The first point cloud containing keypoints.
     * @param cloud2 The second point cloud containing keypoints.
     * @return A matrix representing the transformation between the two clouds.
     */
    Eigen::Matrix4f matchFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2);
};

#endif
