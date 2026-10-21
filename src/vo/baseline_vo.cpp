#include "vo/baseline_vo.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace vo
{
namespace
{

cv::Mat to_cv_camera_matrix(const Eigen::Matrix3d& K)
{
    cv::Mat camera_matrix(3, 3, CV_64F);

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            camera_matrix.at<double>(row, col) = K(row, col);
        }
    }

    return camera_matrix;
}

Eigen::Matrix4d make_pose_matrix(
    const Eigen::Matrix3d& R_world_camera,
    const Eigen::Vector3d& t_world_camera)
{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = R_world_camera;
    pose.block<3, 1>(0, 3) = t_world_camera;
    return pose;
}

bool read_next_gray_frame(dataset::ImageSequence& sequence, cv::Mat& gray)
{
    cv::Mat frame;
    if (!sequence.get_next_frame(frame) || frame.empty())
    {
        return false;
    }

    switch (frame.channels())
    {
        case 1:
            gray = frame;
            break;
        case 3:
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            break;
        case 4:
            cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
            break;
        default:
            return false;
    }

    return !gray.empty();
}

bool estimate_relative_motion(
    const cv::Mat& previous_gray,
    const cv::Mat& current_gray,
    const cv::Mat& camera_matrix,
    const BaselineVOConfig& config,
    Eigen::Matrix3d& R_current_previous,
    Eigen::Vector3d& t_current_previous,
    int& inlier_count)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(previous_gray, keypoints, config.fast_threshold, true);

    if (keypoints.size() < 5)
    {
        return false;
    }

    std::vector<cv::Point2f> points_previous;
    cv::KeyPoint::convert(keypoints, points_previous);

    std::vector<cv::Point2f> points_current;
    std::vector<unsigned char> status;
    std::vector<float> tracking_errors;

    cv::calcOpticalFlowPyrLK(
        previous_gray,
        current_gray,
        points_previous,
        points_current,
        status,
        tracking_errors);

    std::vector<cv::Point2f> good_previous;
    std::vector<cv::Point2f> good_current;
    const std::size_t count = std::min(
        {points_previous.size(), points_current.size(), status.size()});

    for (std::size_t i = 0; i < count; ++i)
    {
        if (status[i] != 0)
        {
            good_previous.push_back(points_previous[i]);
            good_current.push_back(points_current[i]);
        }
    }

    if (good_previous.size() < 5)
    {
        return false;
    }

    cv::Mat inlier_mask;
    cv::Mat essential = cv::findEssentialMat(
        good_previous,
        good_current,
        camera_matrix,
        cv::RANSAC,
        config.ransac_probability,
        config.ransac_threshold_pixels,
        1000,
        inlier_mask);

    if (essential.empty() || essential.rows != 3 || essential.cols != 3)
    {
        return false;
    }

    cv::Mat R_cv;
    cv::Mat t_cv;
    inlier_count = cv::recoverPose(
        essential,
        good_previous,
        good_current,
        camera_matrix,
        R_cv,
        t_cv,
        inlier_mask);

    if (inlier_count < static_cast<int>(config.min_inliers))
    {
        return false;
    }

    cv::Mat R64;
    cv::Mat t64;
    R_cv.convertTo(R64, CV_64F);
    t_cv.convertTo(t64, CV_64F);

    if (R64.rows != 3 || R64.cols != 3 || t64.total() != 3)
    {
        return false;
    }

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            R_current_previous(row, col) = R64.at<double>(row, col);
        }
    }

    t_current_previous <<
        t64.at<double>(0, 0),
        t64.at<double>(1, 0),
        t64.at<double>(2, 0);

    return true;
}

} // namespace

BaselineVOResult run_baseline_vo(
    dataset::KittiDataset& dataset,
    const BaselineVOConfig& config)
{
    if (config.fast_threshold <= 0 ||
        config.ransac_probability <= 0.0 ||
        config.ransac_probability > 1.0 ||
        config.ransac_threshold_pixels <= 0.0 ||
        config.min_inliers < 5)
    {
        throw std::invalid_argument("Invalid baseline VO configuration.");
    }

    dataset::ImageSequence& sequence = dataset.get_image_sequence();
    sequence.reset_stream();

    cv::Mat previous_gray;
    if (!read_next_gray_frame(sequence, previous_gray))
    {
        throw std::runtime_error("Could not read the first image frame.");
    }

    const cv::Mat K = to_cv_camera_matrix(
        dataset.get_calibration().intrinsic_matrix);

    Eigen::Matrix3d R_world_camera = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_world_camera = Eigen::Vector3d::Zero();

    BaselineVOResult result;
    result.poses.push_back(make_pose_matrix(R_world_camera, t_world_camera));

    std::size_t processed_frames = 1;

    while (sequence.has_next_frame() && (config.max_frames == 0 || processed_frames < config.max_frames))
    {
        cv::Mat current_gray;
        if (!read_next_gray_frame(sequence, current_gray))
        {
            throw std::runtime_error(
                "An image frame could not be decoded; stopping to preserve trajectory alignment.");
        }

        ++result.attempted_pairs;

        Eigen::Matrix3d R_current_previous = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_current_previous = Eigen::Vector3d::Zero();
        int inlier_count = 0;

        const bool motion_found = estimate_relative_motion(
            previous_gray,
            current_gray,
            K,
            config,
            R_current_previous,
            t_current_previous,
            inlier_count);

        if (motion_found)
        {
            // recoverPose gives the transform from the previous camera frame
            // to the current camera frame. Invert it to update camera pose in world.
            const Eigen::Matrix3d R_previous_world = R_world_camera;
            const Eigen::Matrix3d R_previous_current = R_current_previous.transpose();

            R_world_camera = R_previous_world * R_previous_current;
            t_world_camera = t_world_camera -
                R_previous_world * R_previous_current * t_current_previous;

            ++result.successful_pairs;
        }
        else
        {
            ++result.skipped_pairs;
        }

        // Keep one output pose per input frame, even when motion estimation fails.
        result.poses.push_back(make_pose_matrix(R_world_camera, t_world_camera));
        previous_gray = current_gray;
        ++processed_frames;
    }

    return result;
}

bool save_kitti_trajectory(
    const std::vector<Eigen::Matrix4d>& poses,
    const std::filesystem::path& output_path)
{
    if (output_path.empty())
    {
        return false;
    }

    std::ofstream output_file(output_path);
    if (!output_file.is_open())
    {
        std::cerr << "Could not open output file: " << output_path.string() << '\n';
        return false;
    }

    output_file << std::fixed << std::setprecision(12);

    for (const Eigen::Matrix4d& pose : poses)
    {
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                output_file << pose(row, col);
                if (!(row == 2 && col == 3))
                {
                    output_file << ' ';
                }
            }
        }
        output_file << '\n';
    }

    if (!output_file)
    {
        std::cerr << "Error while writing trajectory: " << output_path.string() << '\n';
        return false;
    }

    return true;
}

} // namespace vo
