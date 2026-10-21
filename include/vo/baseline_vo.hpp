#pragma once

#include <cstddef>
#include <filesystem>
#include <vector>

#include <Eigen/Dense>

#include "dataset/kitti_dataset.hpp"

namespace vo
{

struct BaselineVOConfig
{
    int fast_threshold = 20;
    double ransac_probability = 0.999;
    double ransac_threshold_pixels = 1.0;
    std::size_t min_inliers = 8;
    std::size_t max_frames = 0;
};

struct BaselineVOResult
{
    std::vector<Eigen::Matrix4d> poses;
    std::size_t attempted_pairs = 0;
    std::size_t successful_pairs = 0;
    std::size_t skipped_pairs = 0;
};

BaselineVOResult run_baseline_vo(
    dataset::KittiDataset& dataset,
    const BaselineVOConfig& config = BaselineVOConfig{});

bool save_kitti_trajectory(
    const std::vector<Eigen::Matrix4d>& poses,
    const std::filesystem::path& output_path);

} // namespace vo
