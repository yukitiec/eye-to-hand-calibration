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
	Utility ut;

public:
	std::filesystem::path path_root, path_img, path_img_left, path_img_right, path_undistort_left, path_undistort_right, path_csv;
	std::string rootDir, imgDir_left, imgDir_right, undistortDir_left, undistortDir_right, csvDir;

	UI_getData() {
		std::cout << "construct UI for getting images and robot tcp position data" << std::endl;
	};

	~UI_getData() {};

	/**
	* @brief move robot to target joints loaded from csv, and save images and robot TCP pose.
	*/
	void main(std::string joints_replay_path);

	/**
	* @brief load target joint poses from csv file.
	* @param[in] path csv file path
	* @return list of 6-dof joint targets
	*/
	std::vector<std::vector<double>> loadPosesCSV(const std::string& path);

	/**
	* @brief make a directory.
	*/
	void makeDir(std::filesystem::path& dirPath);
};