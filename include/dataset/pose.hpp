#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace dataset
{
    using PoseList = std::vector<Eigen::Matrix4d>;

    PoseList load_ground_truth_poses(const std::string& file_path);
}