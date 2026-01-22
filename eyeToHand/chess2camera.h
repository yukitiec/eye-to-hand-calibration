#pragma once

#include "stdafx.h"
#include "global_parameters.h"

class Chess2camera {
private:

    // Define the supported image file extensions
    const std::vector<std::string> imageExtensions = { ".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tiff" };

public:
    //params
//params
    int type_process, width, height;
    double size;
    std::vector<cv::Mat> imgs_left, imgs_right;
    std::vector<cv::Point2d> points_left, points_right;
    std::vector<cv::Mat> tvecs_left, rvecs_left, tvecs_right, rvecs_right;//rotation vector and translation vector.

    std::string winName = "Detected Corners";
    
    //params

    Chess2camera()
    {
        cv::namedWindow(winName);
        std::cout << "construct Chess2camera class" << std::endl;
    }

    void main();

    
    //--- Get corners of chessboard.
    void getCorners(const cv::Mat& frame, std::vector<cv::Point2f>& corners,
        const bool automatic=false);


};