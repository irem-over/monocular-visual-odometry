#pragma once

#include <filesystem>
#include <vector>
#include <string>
#include <cstddef>
#include <opencv2/opencv.hpp>

namespace dataset
{
    namespace fs = std::filesystem;

    std::vector<fs::path> scan_image_directory(const fs::path& directory_path);

    class ImageSequence
    {
    public:
        explicit ImageSequence(std::vector<fs::path> image_paths);

        bool has_next_frame() const;

        std::size_t get_frame_count() const;

        bool get_next_frame(cv::Mat& output_image);

        bool get_frame_by_index(std::size_t frame_index, cv::Mat& output_image) const;

        void reset_stream();

    private:
        std::vector<fs::path> image_paths_;
        std::size_t next_index_ = 0;
    };
}