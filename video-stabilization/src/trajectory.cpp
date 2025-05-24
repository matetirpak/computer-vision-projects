#include <opencv2/opencv.hpp>

#include "trajectory.h"

// Smoothens a trajectory by weighted averaging over a specified window size.
std::vector<std::vector<float>> smoothen_trajectory(
    const std::vector<std::vector<float>>& movements, 
    int window_size) 
{
    int n = movements.size();
    std::vector<std::vector<float>> smoothed;

    float sigma = window_size / 2.0f;

    for (int i = 0; i < n; ++i) {
        int start = std::max(0, i - window_size / 2);
        int end = std::min(n, i + window_size / 2 + 1);

        float weighted_sum_x = 0.0f, weighted_sum_y = 0.0f;
        float total_weight = 0.0f;

        for (int j = start; j < end; ++j) {
            int distance = i - j;
            float weight = std::exp(-(distance * distance) / (2 * sigma * sigma));

            weighted_sum_x += movements[j][0] * weight;
            weighted_sum_y += movements[j][1] * weight;
            total_weight += weight;
        }

        smoothed.push_back({weighted_sum_x / total_weight, weighted_sum_y / total_weight});
    }

    return smoothed;
}

// Given the image pair movements, generates and returns the 2d trajectory of the video.
std::vector<std::vector<float>> generate_trajectory(const std::vector<std::vector<float>>& movements) {
    int n = movements.size();
    std::vector<std::vector<float>> trajectory(n, std::vector<float>(2));
    for (int i = 1; i < n; i++) {
        trajectory[i][0] = trajectory[i-1][0] + movements[i][0];
        trajectory[i][1] = trajectory[i-1][1] + movements[i][1];
    }
    return trajectory;
}

// Generates the x and y offset between each image pair in the video.
std::vector<std::vector<float>> get_movements(std::vector<cv::Mat> video) {
    int n = video.size();
    std::vector<std::vector<float>> ret(n, std::vector<float>(2));
    
    std::vector<cv::KeyPoint> kp_prev;
    std::vector<cv::KeyPoint> kp_cur;
    cv::Mat des_prev;
    cv::Mat des_cur;
    for (int i = 1; i < n; i++) {
        // Get feature matches
        std::vector<cv::DMatch> matches = match_two_images(video[i-1], video[i], kp_prev, kp_cur, des_prev, des_cur, 0.7f);
        // Estimate and save offset
        ret[i] = estimate_translation_ransac(kp_prev, kp_cur, matches);
        // Prepare next iteration
        kp_prev = kp_cur;
        kp_cur.clear();
        des_prev = des_cur.clone();;
    }
    return ret;
}

// Uses ORB to extract image features. Return values are stored in kp and des.
void get_image_features(
    const cv::Mat& img, 
    std::vector<cv::KeyPoint>& kp, 
    cv::Mat& des,
    int n_features)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create(n_features);
    orb->detectAndCompute(img, cv::noArray(), kp, des);
    return;
}

// Generates and returns matches for two images.
std::vector<cv::DMatch> match_two_images(
    const cv::Mat& img1, 
    const cv::Mat& img2, 
    std::vector<cv::KeyPoint>& kp1,
    std::vector<cv::KeyPoint>& kp2,
    cv::Mat& des1, cv::Mat& des2,
    float threshold)
{

    // Get image features if not given already.
    if (kp1.size() == 0) get_image_features(img1, kp1, des1);
    if (kp2.size() == 0) get_image_features(img2, kp2, des2);

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(des1, des2, knn_matches, 2);

    // Apply Lowe's ratio test
    std::vector<cv::DMatch> good_matches;
    for (const auto& m : knn_matches) {
        if (m.size() == 2 && m[0].distance < threshold * m[1].distance) {
            good_matches.push_back(m[0]);
        }
    }

    return good_matches;
}

// Given keypoints and matches returns the x and y offset of the two images.
std::vector<float> estimate_translation_ransac(
    const std::vector<cv::KeyPoint>& kp1,
    const std::vector<cv::KeyPoint>& kp2,
    const std::vector<cv::DMatch>& matches)
{
    // Prepare matched points
    std::vector<cv::Point2f> points1, points2;
    for (const auto& m : matches) {
        points1.push_back(kp1[m.queryIdx].pt);
        points2.push_back(kp2[m.trainIdx].pt);
    }

    cv::Mat inlier_mask;
    cv::Mat affine = cv::estimateAffinePartial2D(points1, points2, inlier_mask, cv::RANSAC);

    if (affine.empty()) {
        // No transform found, return zero translation
        return {0, 0};
    }

    // Collect displacements from inliers
    std::vector<float> dxs, dys;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (inlier_mask.at<uchar>(i)) {
            const auto& p1 = kp1[matches[i].queryIdx].pt;
            const auto& p2 = kp2[matches[i].trainIdx].pt;
            dxs.push_back(p2.x - p1.x);
            dys.push_back(p2.y - p1.y);
        }
    }

    // Median helper
    auto median = [](std::vector<float>& vec) -> float {
        if (vec.empty()) return 0;
        size_t mid = vec.size() / 2;
        std::nth_element(vec.begin(), vec.begin() + mid, vec.end());
        float med = vec[mid];
        if (vec.size() % 2 == 0) {
            std::nth_element(vec.begin(), vec.begin() + mid - 1, vec.end());
            med = 0.5f * (med + vec[mid - 1]);
        }
        return med;
    };

    float tx = median(dxs);
    float ty = median(dys);

    return {tx, ty};
}

// Given two trajectories, calculates the differences.
std::vector<std::vector<float>> calculate_deltas(
    std::vector<std::vector<float>> trajectory, 
    std::vector<std::vector<float>> smoothened)
{
    std::vector<std::vector<float>> deltas;
    for (size_t i = 0; i < trajectory.size(); i++) {
        float dx = trajectory[i][0] - smoothened[i][0];
        float dy = trajectory[i][1] - smoothened[i][1];
        deltas.push_back({dx, dy});
    }

    return deltas;
}
