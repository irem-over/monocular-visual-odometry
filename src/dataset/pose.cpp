#include "dataset/pose.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace dataset
{
    PoseList load_ground_truth_poses(const std::string& file_path)
    {
        std::ifstream input_file(file_path);

        if (!input_file.is_open())
        {
            throw std::runtime_error("Failed to open ground truth poses file: " + file_path);
        }

        PoseList poses;
        std::string line;

        while (std::getline(input_file, line))
        {
            if (line.empty())
            {
                continue;
            }

            std::stringstream line_stream(line);
            double transformation_values[12];

            for (int index = 0; index < 12; ++index)
            {
                line_stream >> transformation_values[index];
            }

            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

            transformation_matrix << transformation_values[0], transformation_values[1], transformation_values[2],  transformation_values[3],
                                     transformation_values[4], transformation_values[5], transformation_values[6],  transformation_values[7],
                                     transformation_values[8], transformation_values[9], transformation_values[10], transformation_values[11],
                                     0.0,                      0.0,                      0.0,                       1.0;

            poses.push_back(transformation_matrix);
        }

        if (poses.empty())
        {
            throw std::runtime_error("No poses could be parsed from file: " + file_path);
        }

        return poses;
    }
}