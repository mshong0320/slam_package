#include "map_manager.h"
#include <cuda_runtime.h>
#include <pcl/common/transforms.h>

/**
 * @brief CUDA kernel to apply a transformation to a point cloud.
 * @param d_points_in Input point cloud in device memory (flattened array).
 * @param d_points_out Transformed point cloud in device memory (flattened array).
 * @param d_transform Transformation matrix in device memory (row-major order).
 * @param num_points Number of points in the input point cloud.
 */
__global__ void transformPointCloudKernel(const float* d_points_in, float* d_points_out, 
                                          const float* d_transform, int num_points) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < num_points) {
        float x = d_points_in[idx * 3];
        float y = d_points_in[idx * 3 + 1];
        float z = d_points_in[idx * 3 + 2];

        d_points_out[idx * 3] = d_transform[0] * x + d_transform[1] * y + d_transform[2] * z + d_transform[3];
        d_points_out[idx * 3 + 1] = d_transform[4] * x + d_transform[5] * y + d_transform[6] * z + d_transform[7];
        d_points_out[idx * 3 + 2] = d_transform[8] * x + d_transform[9] * y + d_transform[10] * z + d_transform[11];
    }
}

void MapManager::integratePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Matrix4f& pose) {
    /**
     * @brief Integrates a point cloud into the global map by transforming it using the provided pose matrix.
     */

    int num_points = cloud->points.size();
    std::vector<float> points_in(num_points * 3);
    for (int i = 0; i < num_points; ++i) {
        points_in[i * 3] = cloud->points[i].x;
        points_in[i * 3 + 1] = cloud->points[i].y;
        points_in[i * 3 + 2] = cloud->points[i].z;
    }

    // Allocate device memory and copy data
    float* d_points_in, *d_points_out, *d_transform;
    cudaMalloc(&d_points_in, num_points * 3 * sizeof(float));
    cudaMalloc(&d_points_out, num_points * 3 * sizeof(float));
    cudaMalloc(&d_transform, 12 * sizeof(float));

    cudaMemcpy(d_points_in, points_in.data(), num_points * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_transform, pose.data(), 12 * sizeof(float), cudaMemcpyHostToDevice);

    // Launch kernel to transform point cloud
    int threads_per_block = 256;
    int blocks = (num_points + threads_per_block - 1) / threads_per_block;
    transformPointCloudKernel<<<blocks, threads_per_block>>>(d_points_in, d_points_out, d_transform, num_points);

    // Retrieve transformed point cloud
    std::vector<float> points_out(num_points * 3);
    cudaMemcpy(points_out.data(), d_points_out, num_points * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < num_points; ++i) {
        pcl::PointXYZ point;
        point.x = points_out[i * 3];
        point.y = points_out[i * 3 + 1];
        point.z = points_out[i * 3 + 2];
        transformed_cloud->points.push_back(point);
    }

    *global_map_ += *transformed_cloud;

    // Free device memory
    cudaFree(d_points_in);
    cudaFree(d_points_out);
    cudaFree(d_transform);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapManager::getMap() {
    /**
     * @brief Retrieves the global map.
     * @return The global map as a point cloud.
     */
    return global_map_;
}
