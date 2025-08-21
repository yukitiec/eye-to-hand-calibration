#pragma once

#include "stdafx.h"
#include "utility.h"

extern std::queue<std::array<cv::Mat1b, 2>> queueFrame;
extern std::queue<int> queueFrameIndex;

class UI_getData {
	/**
	* @brief User interface. displaying current images and robot joints angle
	*/
private:
	const std::vector<std::string> jointNames = { "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3" };
	const std::string sign_save = "s";//save sign.
	const std::string sign_quit = "q";//continue sign.
	const double move_max = 0.6;//max joints move.
	Utility ut;

public:
	std::filesystem::path path_root, path_img, path_img_left, path_img_right, path_undistort_left, path_undistort_right, path_csv;
	std::string rootDir, imgDir_left, imgDir_right, undistortDir_left, undistortDir_right, csvDir;

	UI_getData() {
		std::cout << "construct UI for getting images and robot tcp position data" << std::endl;
	};

	~UI_getData() {};

	/**
	* @brief move robot for chessboard is within camera field of view, and save images and robot TCP pose.
	* @param[in] imgDir, csvDir image and csv directory.
	*/
	void main();

	/**
	* @brief move robot according to human inputs.
	* @param[out] jointsValues incrementals of each joint
	*/
	void adjustRobot(std::vector<double>& jointValues);

	/**
	* @brief make a directory.
	*/
	void makeDir(std::filesystem::path& dirPath);

};