#include <iostream>
#include <iomanip>
#include <exception>
#include <opencv2/opencv.hpp>

#include "dataset/kitti_dataset.hpp"

int main()
{
    try
    {
        const std::string base_path = "data/kitti";
        const std::string sequence_id = "00";

        dataset::KittiDataset dataset(base_path, sequence_id);

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "KittiDataset loaded successfully for sequence: " << sequence_id << "\n";
        std::cout << "Total frames: " << dataset.get_total_frames() << "\n";

        const dataset::CameraCalibration& calibration = dataset.get_calibration();
        std::cout << "Camera fx: " << calibration.fx << ", fy: " << calibration.fy << "\n";
        std::cout << "Camera cx: " << calibration.cx << ", cy: " << calibration.cy << "\n\n";

        const dataset::PoseList& poses = dataset.get_ground_truth_poses();
        std::cout << "Initial ground truth translation (x, y, z): " << poses.front()(0, 3) << ", " << poses.front()(1, 3) << ", "<< poses.front()(2, 3) << "\n";

        cv::Mat first_frame;
        if (dataset.get_image_sequence().get_next_frame(first_frame))
        {
            std::cout << "First frame read successfully. Dimensions: " << first_frame.cols << "x" << first_frame.rows << "\n";
        }
        else
        {
            std::cerr << "Failed to read the first frame from dataset stream.\n";
            return 1;
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Error during KittiDataset verification: " << exception.what() << "\n";
        return 1;
    }

    return 0;
}