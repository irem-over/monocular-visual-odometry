#pragma once

#include <string>
#include <filesystem>
#include <memory>
#include <cstddef>

#include "dataset/calibration.hpp"
#include "dataset/kitti_loader.hpp"
#include "dataset/pose.hpp"

namespace dataset
{
    namespace fs = std::filesystem;

    class KittiDataset
    {
    public:
        KittiDataset(const fs::path& base_directory, const std::string& sequence_id = "00");

        const CameraCalibration& get_calibration() const;

        ImageSequence& get_image_sequence();

        const PoseList& get_ground_truth_poses() const;

        std::size_t get_total_frames() const;

    private:
        fs::path base_directory_;
        std::string sequence_id_;
        CameraCalibration calibration_;
        PoseList ground_truth_poses_;
        std::unique_ptr<ImageSequence> image_sequence_;
    };
}