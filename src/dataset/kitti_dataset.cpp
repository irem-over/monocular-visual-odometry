#include "dataset/kitti_dataset.hpp"

#include <stdexcept>
#include <utility>

namespace dataset
{
    KittiDataset::KittiDataset(const fs::path& base_directory, const std::string& sequence_id)
        : base_directory_(base_directory), sequence_id_(sequence_id)
    {
        const fs::path sequence_directory = base_directory_ / "sequences" / sequence_id_;
        const fs::path calibration_path = sequence_directory / "calib.txt";
        const fs::path image_directory = sequence_directory / "image_0";
        const fs::path pose_path = base_directory_ / "poses" / (sequence_id_ + ".txt");

        if (!fs::exists(sequence_directory))
        {
            throw std::runtime_error("Sequence directory does not exist: " + sequence_directory.string());
        }

        if (!fs::exists(calibration_path))
        {
            throw std::runtime_error("Calibration file does not exist: " + calibration_path.string());
        }

        if (!fs::exists(image_directory))
        {
            throw std::runtime_error("Image directory does not exist: " + image_directory.string());
        }

        if (!fs::exists(pose_path))
        {
            throw std::runtime_error("Pose file does not exist: " + pose_path.string());
        }

        calibration_ = load_camera_calibration(calibration_path.string(), "P0");
        ground_truth_poses_ = load_ground_truth_poses(pose_path.string());

        std::vector<fs::path> image_paths = scan_image_directory(image_directory);

        if (image_paths.empty())
        {
            throw std::runtime_error("No images found in directory: " + image_directory.string());
        }

        if (image_paths.size() != ground_truth_poses_.size())
        {
            throw std::runtime_error("Frame count mismatch: image count does not match ground truth pose count.");
        }

        image_sequence_ = std::make_unique<ImageSequence>(std::move(image_paths));
    }

    const CameraCalibration& KittiDataset::get_calibration() const
    {
        return calibration_;
    }

    ImageSequence& KittiDataset::get_image_sequence()
    {
        return *image_sequence_;
    }

    const PoseList& KittiDataset::get_ground_truth_poses() const
    {
        return ground_truth_poses_;
    }

    std::size_t KittiDataset::get_total_frames() const
    {
        return ground_truth_poses_.size();
    }
}