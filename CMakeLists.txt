cmake_minimum_required(VERSION 3.20)
project(monocular_visual_odometry CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(dataset_lib
    src/dataset/kitti_loader.cpp
    src/dataset/calibration.cpp
    src/dataset/pose.cpp
    src/dataset/kitti_dataset.cpp
)

target_include_directories(dataset_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(dataset_lib PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

add_executable(test_kitti_loader
    tests/test_kitti_loader.cpp
)

target_link_libraries(test_kitti_loader PRIVATE
    dataset_lib
)

add_executable(test_calibration
    tests/test_calibration.cpp
)

target_link_libraries(test_calibration PRIVATE
    dataset_lib
)

add_executable(test_pose
    tests/test_pose.cpp
)

target_link_libraries(test_pose PRIVATE
    dataset_lib
)

add_executable(test_kitti_dataset
    tests/test_kitti_dataset.cpp
)

target_link_libraries(test_kitti_dataset PRIVATE
    dataset_lib
)