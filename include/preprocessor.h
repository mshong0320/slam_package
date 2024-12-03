#ifndef PREPROCESSOR_H
#define PREPROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @class Preprocessor
 * @brief A module responsible for preprocessing LIDAR point clouds by removing noise, downsampling, and normalizing.
 */
class Preprocessor {
public:
    /**
     * @brief Applies noise filtering to the input point cloud.
     * This function uses statistical outlier removal to eliminate noise points that are far from their neighbors.
     * @param cloud The input point cloud.
     * @return The filtered point cloud with reduced noise.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief Downsamples the input point cloud using voxel grid filtering.
     * This reduces the number of points in the cloud to speed up subsequent processing.
     * @param cloud The input point cloud.
     * @param leaf_size The voxel size used for downsampling.
     * @return The downsampled point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size);

    /**
     * @brief Normalizes the input point cloud to a common coordinate system.
     * This function applies translation and scaling to standardize the point cloud for further processing.
     * @param cloud The input point cloud.
     * @return The normalized point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr normalize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

#endif
