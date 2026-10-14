#include <iostream>
#include <iomanip>
#include <string>
#include <exception>

#include "dataset/calibration.hpp"

int main()
{
    try
    {
        const std::string calibration_file_path = "data/kitti/sequences/00/calib.txt";
        const std::string target_camera_id = "P0";

        dataset::CameraCalibration camera_calibration = dataset::load_camera_calibration(calibration_file_path, target_camera_id);

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Calibration successfully loaded for camera: " << target_camera_id << "\n";
        std::cout << "fx: " << camera_calibration.fx << "\n";
        std::cout << "fy: " << camera_calibration.fy << "\n";
        std::cout << "cx: " << camera_calibration.cx << "\n";
        std::cout << "cy: " << camera_calibration.cy << "\n\n";

        std::cout << "Intrinsic Matrix (K):\n" << camera_calibration.intrinsic_matrix << "\n";
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Error during calibration test: " << exception.what() << "\n";
        return 1;
    }

    return 0;
}