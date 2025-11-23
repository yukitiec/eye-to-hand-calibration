// C++ Example for Intel RealSense L515 LiDAR Camera:
// Streams RGB, Aligned Depth, and extracts 3D (X, Y, Z) coordinates for a target pixel.
//
// Prerequisites:
// 1. Intel RealSense SDK 2.0 installed.
// 2. OpenCV library installed and linked.
// 3. Compile with the -lrealsense2 and OpenCV libraries.

//#include <iostream>
//#include <sstream>
//#include <iomanip>
//
//#include <librealsense2/rs.hpp> // RealSense SDK
//#include <opencv2/opencv.hpp>   // OpenCV

#include "realsense.h"
#include "global_parameters.h"



void RealSense::push_frame_with_limit(std::queue<std::pair<cv::Mat, rs2::depth_frame>>& queue, const cv::Mat& frame, const rs2::depth_frame& depth_frame)
{
    // Convert input cv::Mat to cv::Mat1b (grayscale)
    
    while (queue.size() >= kMaxRsQueueSize) {
        queue.pop();
    }
    std::pair<cv::Mat, rs2::depth_frame> info{frame,depth_frame};
    queue.push(info);
}



// Function to convert rs2::frame to cv::Mat
// Handles Depth (Z16) and Color (BGR8) formats.
cv::Mat RealSense::frame_to_mat(const rs2::frame& frame) {
    // Get frame dimensions
    auto profile = frame.get_profile();
    auto format = profile.format();
    int width = profile.as<rs2::video_stream_profile>().width();
    int height = profile.as<rs2::video_stream_profile>().height();

    // Check for color stream (BGR8)
    if (format == RS2_FORMAT_BGR8) {
        return cv::Mat(height, width, CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    }
    // Handle RGB8 by converting channel order to BGR (OpenCV default)
    else if (format == RS2_FORMAT_RGB8) {
        cv::Mat rgb(height, width, CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    // Handle RGBA output (e.g., some colorizer configurations)
    else if (format == RS2_FORMAT_RGBA8) {
        cv::Mat rgba(height, width, CV_8UC4, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        return bgr;
    }
    // Check for depth stream (Z16)
    else if (format == RS2_FORMAT_Z16) {
        return cv::Mat(height, width, CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    }
    // Handle other formats (like Y8 for IR, not needed here but good practice)
    else {
        // Fallback for raw data, should not happen for our setup
        throw std::runtime_error("Frame format not supported for cv::Mat conversion.");
    }
}

// Main usage: Get 3D world coordinates for a predefined pixel in the RGB image.
void RealSense::main()
{
    try
    {
        rs2::pipeline pipe;
        rs2::config cfg;

        cfg.enable_stream(RS2_STREAM_DEPTH, _W, _H, RS2_FORMAT_Z16, _FPS);
        cfg.enable_stream(RS2_STREAM_COLOR, _W, _H, RS2_FORMAT_BGR8, _FPS);

        rs2::pipeline_profile profile = pipe.start(cfg);
        cout << "RealSense L515 pipeline started with " << _W << "x" << _H << " @ " << _FPS << " FPS." << endl;

        // Align depth to color stream
        rs2::align align_to_color(RS2_STREAM_COLOR);

        // Intrinsics for color
        rs2::stream_profile color_stream_profile = profile.get_stream(RS2_STREAM_COLOR);
        rs2::video_stream_profile color_video_profile = color_stream_profile.as<rs2::video_stream_profile>();
        //rs2_intrinsics _color_intrinsics = color_video_profile.get_intrinsics();
        _color_intrinsics = color_video_profile.get_intrinsics();
        cout << "Color Intrinsics (fx, fy): (" << _color_intrinsics.fx << ", " << _color_intrinsics.fy << ")" << endl;

        // Predefined pixel (for example: image center)
        const int target_x = _W / 2;
        const int target_y = _H / 2;
        cout << "Predefined point (x, y): (" << target_x << ", " << target_y << ")" << endl;

        // Colorizer for pretty depth display
        rs2::colorizer color_map;

        // Display windows
        namedWindow("RealSense L515 RGB", WINDOW_AUTOSIZE);
        namedWindow("RealSense L515 Depth", WINDOW_AUTOSIZE);

        int counter = 0;
        while (waitKey(1) < 0 && getWindowProperty("RealSense L515 RGB", WND_PROP_AUTOSIZE) >= 0)
        {
            // Wait for a new set of frames
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::frame color_frame = frames.get_color_frame();
            rs2::depth_frame aligned_depth_frame = frames.get_depth_frame();

            Mat color_image = this->frame_to_mat(color_frame);
            Mat depth_image = this->frame_to_mat(aligned_depth_frame);

            //Push images to the queue.
            if (!color_image.empty()) {
                push_frame_with_limit(q_realsense_img, color_image,aligned_depth_frame);
            }

            rs2::frame depth_colored_frame = color_map.colorize(aligned_depth_frame);
            Mat depth_colormap = this->frame_to_mat(depth_colored_frame);

            imshow("RealSense L515 RGB", color_image);
            imshow("RealSense L515 Depth", depth_colormap);

            counter += 1;
        }

        destroyAllWindows();
    }
    catch (const rs2::error& e)
    {
        cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "): " << e.what() << endl;
    }
    catch (const std::exception& e)
    {
        cerr << e.what() << endl;
    }
}