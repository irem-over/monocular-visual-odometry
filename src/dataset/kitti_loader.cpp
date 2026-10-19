#include "dataset/kitti_loader.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

namespace dataset
{
    std::vector<fs::path> scan_image_directory(const fs::path& directory_path)
    {
        if (!fs::exists(directory_path) || !fs::is_directory(directory_path))
        {
            throw std::runtime_error(
                "Image path is not a directory: " + directory_path.string());
        }
        
        std::vector<fs::path> valid_paths;

        for (const auto& entry : fs::directory_iterator(directory_path))
        {
            if (!entry.is_regular_file())
            {
                continue;
            }

            std::string extension = entry.path().extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char character)
            {
                return static_cast<char>(std::tolower(character));
            });

            if (extension == ".png" || extension == ".jpg" || extension == ".jpeg")
            {
                valid_paths.push_back(entry.path());
            }
        }

        std::sort(valid_paths.begin(), valid_paths.end());
        return valid_paths;
    }

    ImageSequence::ImageSequence(std::vector<fs::path> image_paths)
        : image_paths_(std::move(image_paths)), next_index_(0)
    {
    }

    bool ImageSequence::has_next_frame() const
    {
        return next_index_ < image_paths_.size();
    }

    std::size_t ImageSequence::get_frame_count() const
    {
        return image_paths_.size();
    }

    bool ImageSequence::get_next_frame(cv::Mat& output_image)
    {
        output_image.release();

        if (!has_next_frame())
        {
            return false;
        }

        output_image = cv::imread(image_paths_[next_index_].string(), cv::IMREAD_GRAYSCALE);
        ++next_index_;

        return !output_image.empty();
    }

    bool ImageSequence::get_frame_by_index(std::size_t frame_index, cv::Mat& output_image) const
    {
        output_image.release();

        if (frame_index >= image_paths_.size())
        {
            return false;
        }

        output_image = cv::imread(image_paths_[frame_index].string(), cv::IMREAD_GRAYSCALE);

        return !output_image.empty();
    }

    void ImageSequence::reset_stream()
    {
        next_index_ = 0;
    }
}