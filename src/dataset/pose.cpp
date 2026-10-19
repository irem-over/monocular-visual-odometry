#include "dataset/pose.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dataset
{
    PoseList load_ground_truth_poses(const std::string& file_path)
    {
        std::ifstream input_file(file_path);
        if (!input_file.is_open())
        {
            throw std::runtime_error(
                "Failed to open ground truth poses file: " + file_path);
        }

        PoseList poses;
        std::string line;
        std::size_t line_number = 0;

        while (std::getline(input_file, line))
        {
            ++line_number;
            if (line.find_first_not_of(" \t\r\n") == std::string::npos)
            {
                continue;
            }

            std::stringstream line_stream(line);
            std::array<double, 12> values{};

            for (std::size_t i = 0; i < values.size(); ++i)
            {
                if (!(line_stream >> values[i]) || !std::isfinite(values[i]))
                {
                    throw std::runtime_error(
                        "Malformed pose at line " +
                        std::to_string(line_number) + " in " + file_path);
                }
            }
            
            std::string extra_token;
            if (line_stream >> extra_token)
            {
                throw std::runtime_error(
                    "Unexpected extra token at pose line " +
                    std::to_string(line_number));
                }

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 4; ++col)
                {
                    pose(row, col) = values[static_cast<std::size_t>(row * 4 + col)];
                }
            }
        poses.push_back(pose);
        }

        if (poses.empty())
        {
            throw std::runtime_error(
                "No poses could be parsed from file: " + file_path);
        }
        
        return poses;
    }
}