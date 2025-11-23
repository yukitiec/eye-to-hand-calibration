#pragma once
#include "stdafx.h"
#include "global_parameters.h"

using namespace cv;
using namespace std;

class RealSense {
public:
	// Define stream parameters
	const int _W = 640;
	const int _H = 480;
	const int _FPS = 30;

	const size_t kMaxRsQueueSize = 5;

	RealSense() {}

	void push_frame_with_limit(std::queue<std::pair<cv::Mat, rs2::depth_frame>>& queue, const cv::Mat& frame, const rs2::depth_frame& depth_frame);


	Mat frame_to_mat(const rs2::frame& frame);
	void main();
};