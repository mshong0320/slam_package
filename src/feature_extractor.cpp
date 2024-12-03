#include "feature_extractor.h"
#include <cuda_runtime.h>
#include <cublas_v2.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::extractEdgeFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    /**
     * @brief Extracts edge features from the input point cloud using CUDA for parallelized curvature computation.
     * The curvature of each point is computed using a CUDA kernel, and points with high curvature are considered edge features.
     * Edge features are points where the surface curvature exceeds a predefined threshold.
     * This method significantly accelerates the extraction process using GPU computation.
     * 
     * @param cloud The input point cloud to extract features from.
     * @return A point cloud containing the edge features, which are points with high curvature.
     */
    
    const int N = cloud->points.size();
    float* d_points, *d_curvature;
    
    // Allocate memory on the device (GPU) for point data and curvature values
    cudaMalloc(&d_points, N * 3 * sizeof(float));  // Allocate space for X, Y, Z of each point
    cudaMalloc(&d_curvature, N * sizeof(float));   // Allocate space for curvature values

    // Transfer point data from host (CPU) to device (GPU)
    std::vector<float> points(N * 3);
    for (int i = 0; i < N; ++i) {
        points[i * 3] = cloud->points[i].x;
        points[i * 3 + 1] = cloud->points[i].y;
        points[i * 3 + 2] = cloud->points[i].z;
    }

    // Copy points data to GPU memory
    cudaMemcpy(d_points, points.data(), N * 3 * sizeof(float), cudaMemcpyHostToDevice);

    // Set the number of threads per block and calculate the number of blocks needed for the kernel
    const int threads_per_block = 256;
    const int blocks = (N + threads_per_block - 1) / threads_per_block;

    // Launch the CUDA kernel to compute curvature for each point in the cloud
    computeCurvatureKernel<<<blocks, threads_per_block>>>(d_points, d_curvature, N);

    // Copy the curvature results from GPU to CPU memory
    std::vector<float> curvature(N);
    cudaMemcpy(curvature.data(), d_curvature, N * sizeof(float), cudaMemcpyDeviceToHost);

    // Create a new point cloud to store the edge features
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < N; ++i) {
        // Select points with curvature above the threshold as edge features
        if (curvature[i] > 0.1) {  // Threshold for edge features
            edges->points.push_back(cloud->points[i]);
        }
    }

    // Free the allocated memory on the GPU
    cudaFree(d_points);
    cudaFree(d_curvature);
    
    // Return the point cloud containing edge features
    return edges;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::extractPlaneFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    /**
     * @brief Extracts plane features from the input point cloud.
     * This method uses the same process as edge feature extraction, but with different thresholds for detecting plane features.
     * For simplicity, it is currently implemented as a direct call to the `extractEdgeFeatures` method.
     * 
     * @param cloud The input point cloud to extract plane features from.
     * @return A point cloud containing the plane features (currently just extracted as edge features).
     */
    
    // Use the edge feature extraction method for plane features (simplified in this implementation)
    return extractEdgeFeatures(cloud); 
}
