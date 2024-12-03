#include "feature_matcher.h"
#include <pcl/registration/icp.h>
#include <cuda_runtime.h>
#include <cublas_v2.h>

Eigen::Matrix4f FeatureMatcher::matchFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_features,
                                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_features) {
    /**
     * @brief Matches features from two point clouds by applying a feature matching algorithm like ICP.
     * @param cloud1 The first point cloud.
     * @param cloud2 The second point cloud.
     * @return The transformation matrix aligning the two point clouds.
     */
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_features);
    icp.setInputTarget(target_features);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);

    if (icp.hasConverged()) {
        return icp.getFinalTransformation();
    }

    return Eigen::Matrix4f::Identity();
}
