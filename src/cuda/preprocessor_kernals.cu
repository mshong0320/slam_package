#include <cuda_runtime.h>
#include "preprocessor.h"

/**
 * @brief CUDA kernel for downsampling a 3D point cloud using a voxel grid filter.
 * This kernel performs downsampling by selecting points that fall within a specific voxel grid.
 * The voxel grid is determined by the `leaf_size` parameter, and the kernel ensures that points are grouped within a voxel
 * and only one point is kept per voxel.
 * 
 * @param input The input 3D point cloud array, where each point is represented by three consecutive float values (x, y, z).
 * @param output The output array to store the downsampled 3D point cloud.
 * @param input_size The total number of points in the input point cloud.
 * @param leaf_size The size of each voxel in the voxel grid. This defines the resolution of the downsampling process.
 * @param output_size A pointer to the integer representing the number of points in the output point cloud. It is updated atomically.
 */
__global__ void downsampleKernel(const float* input, float* output, int input_size, float leaf_size, int* output_size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= input_size) return;

    // Calculate the voxel size from the leaf size
    float voxel_size = leaf_size * leaf_size * leaf_size;

    // Check if the current point falls within the voxel grid by checking if its coordinates are divisible by the voxel size
    if ((input[idx * 3] / voxel_size) == (int)(input[idx * 3] / voxel_size)) {
        // If the point is inside the voxel, add it to the output array atomically
        int output_idx = atomicAdd(output_size, 1);
        output[output_idx * 3] = input[idx * 3];
        output[output_idx * 3 + 1] = input[idx * 3 + 1];
        output[output_idx * 3 + 2] = input[idx * 3 + 2];
    }
}
