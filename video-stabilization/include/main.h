#ifndef MAIN_H
#define MAIN_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

int main();

std::vector<cv::Mat> stabilize_video(std::vector<cv::Mat> video, int window_size, int lowe_ratio, float zoom_percent, bool visualize);

std::vector<cv::Mat> apply_movement_to_video(
    const std::vector<cv::Mat>& video,
    const std::vector<std::vector<float>>& movements
);

#endif
