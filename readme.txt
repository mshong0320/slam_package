SLAM Package

This repository contains a modular SLAM (Simultaneous Localization and Mapping) system that integrates multiple components like data preprocessing, feature extraction, pose estimation, map management, and loop closure. The system can be integrated both as a standalone application or into a ROS2-based robot framework.

Table of Contents
-----------------
- Overview
- Features
- Structure
- Installation
- Usage
  - Standalone Usage
  - ROS2 Integration
- Dependencies
- Licenses

Overview
--------
The SLAM package integrates several key components:

- Preprocessor: Downsamples and filters point clouds for further processing.
- Feature Extractor: Detects features (edges, planes, etc.) from point clouds.
- Feature Matcher: Matches features from consecutive point clouds to estimate relative transformations.
- Pose Estimator: Fuses transformations from odometry and feature matching using an Extended Kalman Filter (EKF).
- Map Manager: Updates and manages the global map of the environment.
- Loop Closure: Detects potential loop closures in the trajectory and optimizes the pose graph.

The system can be used in a standalone mode or as part of a ROS2 node.

Features
--------
- Efficient point cloud processing with downsampling and feature extraction.
- Pose estimation using EKF for combining odometry and feature-based transforms.
- Map management with continuous updating of the global map.
- Loop closure detection to optimize the trajectory and correct drift.
- CUDA-enabled kernels for efficient point cloud processing on the GPU.

Structure
---------
The repository follows the structure outlined below:

slam_package/
├── CMakeLists.txt          # CMake configuration for building the package
├── README.md               # This README file
├── include/                # Header files for SLAM modules
│   ├── preprocessor.h      # Preprocessor class for point cloud filtering
│   ├── feature_extractor.h # FeatureExtractor class for detecting features
│   ├── feature_matcher.h   # FeatureMatcher class for matching features
│   ├── pose_estimator.h    # PoseEstimator class for pose fusion using EKF
│   ├── map_manager.h       # MapManager class for managing global map
│   └── loop_closure.h      # LoopClosure class for detecting and optimizing loop closures
├── src/                    # Source files for SLAM modules
│   ├── preprocessor.cpp    # Implementation of Preprocessor
│   ├── feature_extractor.cpp # Implementation of FeatureExtractor
│   ├── feature_matcher.cpp # Implementation of FeatureMatcher
│   ├── pose_estimator.cpp  # Implementation of PoseEstimator
│   ├── map_manager.cpp     # Implementation of MapManager
│   ├── loop_closure.cpp    # Implementation of LoopClosure
│   ├── main.cpp            # Main entry for standalone usage
│   └── ros2_integration.cpp # ROS2 integration for handling messages
└── src/cuda/               # CUDA-specific code for GPU processing
    ├── preprocessor_kernels.cu # CUDA kernel for point cloud downsampling
    └── feature_kernels.cu    # CUDA kernel for feature extraction

Key Files
---------
- main.cpp: Entry point for standalone usage, where the SLAM pipeline is run offline on a set of point clouds.
- ros2_integration.cpp: Integrates SLAM with ROS2, subscribing to sensor topics (PointCloud2, Odometry) and publishing results.
- CMakeLists.txt: CMake configuration to compile and link the SLAM package, along with CUDA and PCL dependencies.

Installation
------------
To build and use the slam_package, follow the instructions below.

### Prerequisites
- **CUDA Toolkit**: Ensure you have the CUDA Toolkit installed for GPU acceleration.
- **PCL (Point Cloud Library)**: Install PCL by following the installation instructions.
- **ROS2 (Optional)**: If you plan to integrate with ROS2, ensure you have a working ROS2 setup. Follow the ROS2 installation guide.

### Building the Package
1. Clone the repository:
    ```
    git clone https://github.com/yourusername/slam_package.git
    cd slam_package
    ```

2. Create a build directory:
    ```
    mkdir build
    cd build
    ```

3. Configure the package using CMake:
    ```
    cmake ..
    ```

4. Build the package:
    ```
    make
    ```

5. (Optional) If you are using ROS2, ensure to build using ROS2's workspace setup (`colcon build`).

Usage
------
### Standalone Usage
To use the package in a standalone manner (i.e., without ROS2), compile and run the `main.cpp` file. This will process point clouds from a specified directory and update the global map.

1. Prepare a directory with `.pcd` files (Point Cloud Data files).
2. Edit the dataset directory in `main.cpp` to point to your dataset:
    ```cpp
    std::string dataset_dir = "/path/to/your/pcd_files/";
    ```
3. Run the executable:
    ```
    ./slam_package
    ```
4. Output: The final global map will be saved as `global_map.pcd`.

### ROS2 Integration
If you want to integrate the SLAM system into a ROS2-based robot, follow the steps below.

1. Modify the ROS2 integration code in `ros2_integration.cpp` to subscribe to your desired ROS2 topics (e.g., PointCloud2, Odometry).
2. Build the package using ROS2:
    ```
    colcon build --symlink-install
    ```
3. Run the ROS2 node:
    ```
    ros2 run slam_package slam_node
    ```

4. **ROS2 Topics**:
   - Subscribe to the `PointCloud2` and `Odometry` topics to process incoming data.
   - Publish the results to any custom or predefined topics as needed.

Dependencies
------------
This project depends on the following libraries:
- **PCL (Point Cloud Library)**: For point cloud processing.
- **CUDA**: For GPU-accelerated kernels.
- **Eigen**: For matrix operations and transformations.
- **ROS2 (Optional)**: For integration with ROS2.

Licenses
--------
This project is licensed under the MIT License. See the LICENSE file for details.

