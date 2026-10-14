#pragma once

#include <string>
#include <Eigen/Dense>

namespace dataset
{
    struct CameraCalibration
    {
        double fx;
        double fy;
        double cx;
        double cy;
        Eigen::Matrix3d intrinsic_matrix;
    };

    CameraCalibration load_camera_calibration(const std::string& file_path, const std::string& camera_id = "P0");
}