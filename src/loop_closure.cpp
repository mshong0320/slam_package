#include "loop_closure.h"
#include <cuda_runtime.h>
#include <cublas_v2.h>

/**
 * @brief CUDA kernel to compute distances between a scan point and map points.
 * @param d_current_scan Input scan points in device memory.
 * @param d_map Map points in device memory.
 * @param d_distances Computed distances between scan points and nearest map points.
 * @param num_scan_points Number of points in the scan.
 * @param num_map_points Number of points in the map.
 */
__global__ void computeDistanceKernel(const float* d_current_scan, const float* d_map, 
                                      float* d_distances, int num_scan_points, int num_map_points) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < num_scan_points) {
        float min_dist = 1e9;
        float x1 = d_current_scan[idx * 3];
        float y1 = d_current_scan[idx * 3 + 1];
        float z1 = d_current_scan[idx * 3 + 2];

        for (int j = 0; j < num_map_points; ++j) {
            float x2 = d_map[j * 3];
            float y2 = d_map[j * 3 + 1];
            float z2 = d_map[j * 3 + 2];
            float dist = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        d_distances[idx] = min_dist;
    }
}

Eigen::Matrix4f LoopClosure::detectLoop(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_scan,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& map) {
	/**
     * @brief Detects a loop closure by calculating nearest neighbors between current scan and map.
     * @return The estimated transformation matrix if loop closure is detected.
     */
    int num_scan_points = current_scan->points.size();
    int num_map_points = map->points.size();

    float* d_current_scan, *d_map, *d_distances;
    cudaMalloc(&d_current_scan, num_scan_points * 3 * sizeof(float));
    cudaMalloc(&d_map, num_map_points * 3 * sizeof(float));
    cudaMalloc(&d_distances, num_scan_points * sizeof(float));

    std::vector<float> scan_data(num_scan_points * 3);
    std::vector<float> map_data(num_map_points * 3);

    for (int i = 0; i < num_scan_points; ++i) {
        scan_data[i * 3] = current_scan->points[i].x;
        scan_data[i * 3 + 1] = current_scan->points[i].y;
        scan_data[i * 3 + 2] = current_scan->points[i].z;
    }

    for (int i = 0; i < num_map_points; ++i) {
        map_data[i * 3] = map->points[i].x;
        map_data[i * 3 + 1] = map->points[i].y;
        map_data[i * 3 + 2] = map->points[i].z;
    }

    cudaMemcpy(d_current_scan, scan_data.data(), num_scan_points * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_map, map_data.data(), num_map_points * 3 * sizeof(float), cudaMemcpyHostToDevice);

    int threads_per_block = 256;
    int blocks = (num_scan_points + threads_per_block - 1) / threads_per_block;
    computeDistanceKernel<<<blocks, threads_per_block>>>(d_current_scan, d_map, d_distances, num_scan_points, num_map_points);

    std::vector<float> distances(num_scan_points);
    cudaMemcpy(distances.data(), d_distances, num_scan_points * sizeof(float), cudaMemcpyDeviceToHost);

    // Perform nearest neighbor matching to estimate loop closure transformation (placeholder)
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    cudaFree(d_current_scan);
    cudaFree(d_map);
    cudaFree(d_distances);

    return transformation;
}
