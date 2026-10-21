#include "vo/baseline_vo.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    const std::filesystem::path dataset_root =
        (argc > 1) ? argv[1] : "data/kitti";

    const std::string sequence_id =
        (argc > 2) ? argv[2] : "00";

    std::filesystem::path output_path =
        "results/sequence_" + sequence_id + "_baseline.txt";

    if (argc > 3)
    {
        output_path = argv[3];
    }

    try
    {
        dataset::KittiDataset kitti_dataset(dataset_root, sequence_id);

        std::cout << "Sequence: " << sequence_id << "\n";
        std::cout << "Frames: "
                  << kitti_dataset.get_total_frames()
                  << "\n";
        vo::BaselineVOConfig config;
        config.max_frames = 100;

        const vo::BaselineVOResult result = vo::run_baseline_vo(kitti_dataset, config);

        if (!vo::save_kitti_trajectory(result.poses, output_path))
        {
            std::cerr << "Failed to save trajectory.\n";
            return 1;
        }

        std::cout << "Attempted frame pairs: "
                  << result.attempted_pairs << "\n";

        std::cout << "Successful pose estimation: "
                  << result.successful_pairs << "\n";

        std::cout << "Skipped pose estimation: "
                  << result.skipped_pairs << "\n";

        std::cout << "Saved poses: "
                  << result.poses.size() << "\n";

        std::cout << "Trajectory: "
                  << output_path << "\n";
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Baseline VO failed: "
                  << exception.what() << "\n";

        return 1;
    }

    return 0;
}