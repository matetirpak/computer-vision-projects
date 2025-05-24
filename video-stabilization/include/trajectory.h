#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <opencv2/opencv.hpp>
#include <vector>

std::vector<std::vector<float>> smoothen_trajectory(
    const std::vector<std::vector<float>>& movements, 
    int window_size = 5
);

std::vector<std::vector<float>> generate_trajectory(const std::vector<std::vector<float>>& movements);

std::vector<std::vector<float>> get_movements(std::vector<cv::Mat> video);
void get_image_features(
    const cv::Mat& img, 
    std::vector<cv::KeyPoint>& kp, 
    cv::Mat& des,
    int n_features = 1500
);

std::vector<cv::DMatch> match_two_images(
    const cv::Mat& img1, 
    const cv::Mat& img2, 
    std::vector<cv::KeyPoint>& kp1,
    std::vector<cv::KeyPoint>& kp2,
    cv::Mat& des1, cv::Mat& des2,
    float threshold = 0.7f
);

std::vector<float> estimate_translation_ransac(
    const std::vector<cv::KeyPoint>& kp1,
    const std::vector<cv::KeyPoint>& kp2,
    const std::vector<cv::DMatch>& matches
);

std::vector<std::vector<float>> calculate_deltas(
    std::vector<std::vector<float>> trajectory, 
    std::vector<std::vector<float>> smoothened
);

#endif
