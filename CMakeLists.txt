cmake_minimum_required(VERSION 3.10)
project(slam_package)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)
find_package(rclcpp REQUIRED)  # ROS2 core library
find_package(sensor_msgs REQUIRED)  # For point cloud messages
find_package(nav_msgs REQUIRED)  # For odometry messages
find_package(Eigen3 REQUIRED)  # Eigen for matrix operations

# Include Directories
include_directories(${PCL_INCLUDE_DIRS} include)

# CUDA-specific flags
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=sm_60")  # Adjust according to your target GPU architecture

# Source Files
set(SOURCES
    src/preprocessor.cpp
    src/feature_extractor.cpp
    src/feature_matcher.cpp
    src/pose_estimator.cpp
    src/map_manager.cpp
    src/loop_closure.cpp
)

# CUDA source files
set(CUDA_SOURCES
    src/cuda/preprocessor_kernels.cu
    src/cuda/feature_kernels.cu
)

# Add CUDA kernel library
add_library(cuda_kernels ${CUDA_SOURCES})
target_include_directories(cuda_kernels PUBLIC ${CUDA_INCLUDE_DIRS})

# Add executable
add_executable(slam_package src/main.cpp src/ros2_integration.cpp)

# Link libraries
target_link_libraries(slam_package
    preprocessor
    feature_extractor
    feature_matcher
    pose_estimator
    map_manager
    loop_closure
    cuda_kernels
    ${PCL_LIBRARIES}
    ${CUDA_LIBRARIES}
    rclcpp::rclcpp      # ROS2 core library
    sensor_msgs::sensor_msgs  # ROS2 point cloud message
    nav_msgs::nav_msgs  # ROS2 odometry message
    Eigen3::Eigen       # Eigen for matrix operations
)

# Additional CMake settings
set_target_properties(slam_package PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# Optional: Enable CUDA for .cu files
# If needed, uncomment the following line to explicitly use CUDA for compilation
# cuda_add_library(cuda_kernels ${CUDA_SOURCES})
