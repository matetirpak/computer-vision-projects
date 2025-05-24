#include <cstdio>
#include <opencv2/opencv.hpp>
#include <vector>

#include "trajectory.h"
#include "visualizations.h"

// Displays a video in 30 fps.
void play_video(std::vector<cv::Mat> video) {
    // Show each frame for 33ms. Close if any key pressed.
    for (cv::Mat frame : video) {
        cv::imshow("Video", frame);
        if (cv::waitKey(33) >= 0) break;
    }
    cv::destroyAllWindows();
}

// Saves the video to the disk.
void save_video_as_mp4(const std::vector<cv::Mat>& video, const std::string& filename, int fps) {
    if (video.empty()) {
        std::cerr << "Video is empty!\n";
        return;
    }

    cv::Size frame_size(video[0].cols, video[0].rows);
    cv::VideoWriter writer(filename, cv::VideoWriter::fourcc('m','p','4','v'), fps, frame_size);

    if (!writer.isOpened()) {
        std::cerr << "Failed to open video writer!\n";
        return;
    }

    for (const auto& frame : video) {
        if (frame.size() != frame_size || frame.type() != CV_8UC3) {
            std::cerr << "Frame size/type mismatch!\n";
            continue;
        }
        writer.write(frame);
    }

    writer.release();
    std::cout << "Video saved as: " << filename << "\n";
}

// Generates and adds keypoints to a video.
std::vector<cv::Mat> add_keypoints(const std::vector<cv::Mat>& video) {
    std::vector<cv::Mat> ret;
    for (const auto& frame : video) {
        // Get keypoints
        std::vector<cv::KeyPoint> kp;
        cv::Mat des;
        get_image_features(frame, kp, des);

        // Draw keypoints onto a copy of the frame
        cv::Mat frame_output;
        cv::drawKeypoints(frame, kp, frame_output, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

        ret.push_back(frame_output);
    }

    return ret;
}

// Displays a graph for the x and y trajectory.
void plot_trajectory(std::vector<std::vector<float>> trajectory) {
    int n = trajectory.size();
    
    FILE* gnuplot = popen("gnuplot -persistent", "w");
    fprintf(gnuplot, "plot '-' with lines title 'X', '-' with lines title 'Y'\n");

    // Plot x
    for (size_t i = 0; i < n; i++)
        fprintf(gnuplot, "%zu %f\n", i, trajectory[i][0]);

    fprintf(gnuplot, "e\n");
    
    // Plot y
    for (size_t i = 0; i < n; i++)
        fprintf(gnuplot, "%zu %f\n", i, trajectory[i][1]);

    fprintf(gnuplot, "e\n");
    
    fflush(gnuplot);
    pclose(gnuplot);

    return;
}