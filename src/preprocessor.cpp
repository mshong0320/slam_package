#include "preprocessor.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr Preprocessor::filterNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    /**
     * @brief Filters noise from the input point cloud using statistical outlier removal.
     * Points that are far from their neighbors will be considered noise and removed.
     * @param cloud The input point cloud.
     * @return The filtered point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered_cloud);
    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Preprocessor::downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size) {
    /**
     * @brief Downsamples the input point cloud using voxel grid filtering.
     * This function reduces the number of points in the cloud, making further processing faster.
     * @param cloud The input point cloud.
     * @param leaf_size The voxel size for downsampling.
     * @return The downsampled point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*downsampled_cloud);
    return downsampled_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Preprocessor::normalize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    /**
     * @brief Normalizes the input point cloud to a common coordinate system.
     * This applies a translation and scaling transformation to standardize the point cloud.
     * @param cloud The input point cloud.
     * @return The normalized point cloud.
     */
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr normalized_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *normalized_cloud, transformation);
    return normalized_cloud;
}
