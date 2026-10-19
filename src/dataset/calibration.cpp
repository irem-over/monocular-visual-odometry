#include "dataset/calibration.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dataset
{
    CameraCalibration load_camera_calibration(
        const std::string& file_path,
        const std::string& camera_id)
    {
        std::ifstream input_file(file_path);
        if (!input_file.is_open())
        {
            throw std::runtime_error(
                "Failed to open calibration file: " + file_path);
        }

        std::string line;
        std::size_t line_number = 0;

        while (std::getline(input_file, line))
        {
            ++line_number;
            std::stringstream line_stream(line);
            std::string tag;

            if (!(line_stream >> tag))
            {
                continue;
            }

            if (tag != camera_id + ":")
            {
                continue;
            }

            std::array<double, 12> values{};
            for (std::size_t i = 0; i < values.size(); ++i)
            {
                if (!(line_stream >> values[i]) || !std::isfinite(values[i]))
                {
                    throw std::runtime_error(
                        "Malformed calibration values at line " +
                        std::to_string(line_number) + " in " + file_path);
                }
            }

            std::string extra_token;
            if (line_stream >> extra_token)
            {
                throw std::runtime_error(
                    "Unexpected extra token at calibration line " +
                    std::to_string(line_number));
            }

            CameraCalibration calibration{};
            calibration.fx = values[0];
            calibration.cx = values[2];
            calibration.fy = values[5];
            calibration.cy = values[6];

            if (calibration.fx <= 0.0 || calibration.fy <= 0.0)
            {
                throw std::runtime_error(
                    "Invalid focal length in calibration file: " + file_path);
            }

            calibration.intrinsic_matrix <<
            calibration.fx, 0.0, calibration.cx,
            0.0, calibration.fy, calibration.cy,
            0.0, 0.0, 1.0;
            return calibration;
        }

        throw std::runtime_error(
            "Camera ID not found in calibration file: " + camera_id);
    }
}