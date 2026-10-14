#include <iostream>
#include <iomanip>
#include <string>
#include <exception>

#include "dataset/pose.hpp"

int main()
{
    try
    {
        const std::string pose_file_path = "data/kitti/poses/00.txt";
        dataset::PoseList poses = dataset::load_ground_truth_poses(pose_file_path);

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Successfully loaded poses. Total count: " << poses.size() << "\n\n";

        std::cout << "Initial Pose (Frame 0):\n" << poses.front() << "\n\n";
        std::cout << "Final Pose (Frame " << poses.size() - 1 << "):\n" << poses.back() << "\n";
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Error during pose test: " << exception.what() << "\n";
        return 1;
    }

    return 0;
}