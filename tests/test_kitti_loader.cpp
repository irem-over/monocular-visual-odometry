#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "dataset/kitti_loader.hpp"

namespace fs = std::filesystem;

int main()
{
    const fs::path dataset_path = "data/kitti/sequences/00/image_0";

    if (!fs::exists(dataset_path) || !fs::is_directory(dataset_path))
    {
        std::cerr << "Dataset directory not found: " << dataset_path << "\n";
        return 1;
    }

    std::vector<fs::path> image_paths = dataset::scan_image_directory(dataset_path);

    if (image_paths.empty())
    {
        std::cerr << "No valid images found in directory: " << dataset_path << "\n";
        return 1;
    }

    std::cout << "Loaded " << image_paths.size() << " frames from " << dataset_path << "\n";

    dataset::ImageSequence sequence(std::move(image_paths));
    cv::Mat current_frame;

    while (sequence.has_next_frame())
    {
        if (!sequence.get_next_frame(current_frame))
        {
            std::cerr << "Failed to read frame, skipping.\n";
            continue;
        }

        cv::imshow("KITTI Stream Verification", current_frame);

        int key_pressed = cv::waitKey(15);
        if (key_pressed == 27)
        {
            std::cout << "Playback terminated by user.\n";
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}