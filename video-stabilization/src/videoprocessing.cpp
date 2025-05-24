#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <filesystem>

#include "videoprocessing.h"

namespace fs = std::filesystem;

// Applies zoom onto an image.
cv::Mat zoom_in(const cv::Mat& image, float zoom_percent) {
    float scale = 1.0f + zoom_percent / 100.0f;

    int new_width = static_cast<int>(image.cols / scale);
    int new_height = static_cast<int>(image.rows / scale);

    int x1 = (image.cols - new_width) / 2;
    int y1 = (image.rows - new_height) / 2;

    cv::Rect roi(x1, y1, new_width, new_height);
    cv::Mat cropped = image(roi);

    cv::Mat zoomed;
    cv::resize(cropped, zoomed, image.size(), 0, 0, cv::INTER_LANCZOS4);

    return zoomed;
}

void apply_zoom_to_video(std::vector<cv::Mat>& video, float zoom_percent) {
    for (int i = 0; i < video.size(); i++) {
        video[i] = zoom_in(video[i], zoom_percent);
    }
    return;
}

// Given a directory of numbered images, returns them as a list of matrices.
std::vector<cv::Mat> extract_video(std::string path) {
    std::string directory = path; // Replace with your directory path
    std::vector<std::string> image_files;

    // Collect all image file paths
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            image_files.push_back(entry.path().string());
        }
    }
    std::sort(image_files.begin(), image_files.end());


    // Iterate and store all frames for return
    std::vector<cv::Mat> ret;
    for (const auto& file : image_files) {
        cv::Mat frame = cv::imread(file);
        if (frame.empty()) {
            std::cerr << "Could not read the image: " << file << std::endl;
            continue;
        }
        ret.push_back(frame);
    }
    return ret;
}
