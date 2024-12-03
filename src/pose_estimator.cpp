#include "pose_estimator.h"
#include <cuda_runtime.h>
#include <cublas_v2.h>

/**
 * @brief Estimates the pose of the system using an Extended Kalman Filter (EKF) approach.
 * This function fuses two different transformations—one from odometry and another from feature-based
 * measurements—using a simple weighted average. The fusing process is accelerated by cuBLAS for matrix operations.
 * 
 * The fusion is done using a matrix addition where each transformation (odom_transform and feature_transform) is weighted
 * by a coefficient (alpha and beta). The result is stored in the fused_pose.
 * 
 * This method assumes that both transformations are 4x4 matrices representing homogeneous coordinates.
 * 
 * @param odom_transform The transformation matrix obtained from the odometry information.
 * @param feature_transform The transformation matrix derived from feature-based measurements.
 * 
 * @return A 4x4 matrix representing the fused pose after combining both odometry and feature-based transformations.
 */
Eigen::Matrix4f PoseEstimator::estimatePoseWithEKF(const Eigen::Matrix4f& odom_transform,
                                                   const Eigen::Matrix4f& feature_transform) {
    Eigen::Matrix4f fused_pose;

    // CUDA cuBLAS for matrix addition
    cublasHandle_t handle;
    cublasCreate(&handle);

    float alpha = 0.5f;
    float beta = 0.5f;
    cublasSgeam(handle, CUBLAS_OP_N, CUBLAS_OP_N,
                4, 4,
                &alpha, odom_transform.data(), 4,
                &beta, feature_transform.data(), 4,
                fused_pose.data(), 4);

    cublasDestroy(handle);
    return fused_pose;
}
