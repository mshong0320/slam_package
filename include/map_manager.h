#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

/**
 * @class MapManager
 * @brief Manages the integration and retrieval of the global map built from LIDAR scans.
 */
class MapManager {
public:
    /**
     * @brief Integrates a point cloud into the global map using a given pose transformation.
     * @param cloud The input point cloud to be integrated.
     * @param pose The 4x4 transformation matrix representing the pose.
     */
    void integratePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Matrix4f& pose);

    /**
     * @brief Retrieves the global map as a point cloud.
     * @return The global map represented as a point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getMap();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_; ///< The global map represented as a point cloud.
};

#endif
