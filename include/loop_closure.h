#ifndef LOOP_CLOSURE_H
#define LOOP_CLOSURE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

/**
 * @class LoopClosure
 * @brief Detects and handles loop closures in SLAM using 3D point cloud scans.
 */
class LoopClosure {
public:
    /**
     * @brief Detects a loop closure by comparing the current scan with the global map.
     * @param current_scan The current point cloud scan.
     * @param map The global map point cloud.
     * @return The transformation matrix to align the current scan with the map, if a loop closure is detected.
     */
    Eigen::Matrix4f detectLoop(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_scan,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& map);
};

#endif
