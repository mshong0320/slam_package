#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <Eigen/Dense>

/**
 * @class PoseEstimator
 * @brief A class for estimating the pose of the system using various filtering and fusion techniques.
 * 
 * This class provides methods to estimate the pose based on sensor inputs, such as odometry and feature-based measurements.
 * It uses an Extended Kalman Filter (EKF) approach to combine these transformations into a final fused pose.
 */
class PoseEstimator {
public:
    /**
     * @brief Estimates the pose of the system using an Extended Kalman Filter (EKF) approach.
     * 
     * This method fuses two transformation matrices (one from odometry and one from feature-based measurements)
     * using a weighted average. The matrix addition is accelerated by cuBLAS for high-performance computation.
     * The result is a 4x4 transformation matrix representing the fused pose.
     * 
     * @param odom_transform A 4x4 transformation matrix obtained from odometry.
     * @param feature_transform A 4x4 transformation matrix derived from feature-based measurements.
     * 
     * @return A 4x4 matrix representing the fused pose after combining both odometry and feature-based transformations.
     */
    Eigen::Matrix4f estimatePoseWithEKF(const Eigen::Matrix4f& odom_transform, const Eigen::Matrix4f& feature_transform);
};

#endif
