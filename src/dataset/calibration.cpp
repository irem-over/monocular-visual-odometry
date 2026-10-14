#include "dataset/calibration.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace dataset
{
    CameraCalibration load_camera_calibration(const std::string& file_path, const std::string& camera_id)
    {
        std::ifstream input_file(file_path);

        if (!input_file.is_open())
        {
            throw std::runtime_error("Failed to open calibration file: " + file_path);
        }

        std::string line;

        while (std::getline(input_file, line))
        {
            std::stringstream line_stream(line);
            std::string tag;
            line_stream >> tag;

            if (tag == camera_id + ":")
            {
                double projection_values[12];

                for (int index = 0; index < 12; ++index)
                {
                    line_stream >> projection_values[index];
                }

                CameraCalibration calibration;

                calibration.fx = projection_values[0];
                calibration.cx = projection_values[2];
                calibration.fy = projection_values[5];
                calibration.cy = projection_values[6];

                calibration.intrinsic_matrix << calibration.fx, 0.0,            calibration.cx,
                                                0.0,            calibration.fy, calibration.cy,
                                                0.0,            0.0,            1.0;

                return calibration;
            }
        }

        throw std::runtime_error("Camera ID not found in calibration file: " + camera_id);
    }
}