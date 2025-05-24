#ifndef VISUALIZATIONS_H
#define VISUALIZATIONS_H

#include <opencv2/opencv.hpp>
#include <vector>

void play_video(std::vector<cv::Mat> video);

void save_video_as_mp4(const std::vector<cv::Mat>& video, const std::string& filename, int fps = 30);

std::vector<cv::Mat> add_keypoints(const std::vector<cv::Mat>& video);

void plot_trajectory(std::vector<std::vector<float>> movements);

#endif
