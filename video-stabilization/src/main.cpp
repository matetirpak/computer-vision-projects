#include <cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>

#include "main.h"
#include "trajectory.h"
#include "videoprocessing.h"
#include "visualizations.h"

namespace fs = std::filesystem;

// Parameters
std::string images_path = "cooler/cam2/";
std::string output_dir = "demos/";
bool g_visualize = true;


int main() {
    // Modify extract_video according to how the video is saved
    std::vector<cv::Mat> video = extract_video(images_path);
    auto stabilized = stabilize_video(video, 7, 0.6f, 4.0f, g_visualize);
    
    // Save videos
    save_video_as_mp4(video, output_dir + "original.mp4", 30);
    save_video_as_mp4(stabilized, output_dir + "stabilized.mp4", 30);

    // Demos
    if (g_visualize) {
        play_video(video);
        play_video(stabilized);
    }
    
    return 0;
}

// All in one function. Given a video and parameters, returns it with its shake stabilized.
std::vector<cv::Mat> stabilize_video(std::vector<cv::Mat> video, int window_size, int lowe_ratio, float zoom_percent, bool visualize) {
    std::vector<std::vector<float>> movements = get_movements(video);
    std::vector<std::vector<float>> trajectory = generate_trajectory(movements);
    std::vector<std::vector<float>> smoothened = smoothen_trajectory(trajectory, window_size);
    if (visualize) {
        plot_trajectory(trajectory);
        plot_trajectory(smoothened);
    }
    std::vector<std::vector<float>> deltas = calculate_deltas(trajectory, smoothened);
    std::vector<cv::Mat> stabilized_video = apply_movement_to_video(video, deltas);
    apply_zoom_to_video(stabilized_video, zoom_percent);

    return stabilized_video;
}

// Shifts video frames according to the offset.
std::vector<cv::Mat> apply_movement_to_video(
    const std::vector<cv::Mat>& video,
    const std::vector<std::vector<float>>& movements)
{
    std::vector<cv::Mat> stabilized;
    int n = std::min(video.size(), movements.size());

    for (int i = 0; i < n; ++i) {
        float dx = movements[i][0];
        float dy = movements[i][1];

        cv::Mat T = (cv::Mat_<double>(2, 3) << 1, 0, -dx, 0, 1, -dy);

        cv::Mat stabilized_frame;
        cv::warpAffine(video[i], stabilized_frame, T, video[i].size());

        stabilized.push_back(stabilized_frame);
    }

    return stabilized;
}
