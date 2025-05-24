#ifndef VIDEOPROCESSING_H
#define VIDEOPROCESSING_H

#include <string>
#include <vector>

cv::Mat zoom_in(const cv::Mat& image, float zoom_percent);

void apply_zoom_to_video(std::vector<cv::Mat>& video, float zoom_percent);

std::vector<cv::Mat> extract_video(std::string path);

#endif
