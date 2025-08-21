#pragma once

#include "stdafx.h"
#include "ui_getData.h"
#include "triangulation.h"
#include "chess2camera.h"

/* frame queue */
extern std::queue<std::array<cv::Mat1b, 2>> queueFrame;
extern std::queue<int> queueFrameIndex;

extern std::queue<bool> q_startTracking; //start tracking
extern std::queue<bool> q_endTracking; //end tracking

class Calibration {
	/**
	* @brief eye to hand calibration class.
	*/
private:
	//constructor
	UI_getData ui;//get robot tcp pose and images.
	const cv::Mat H_tcp2chess = (cv::Mat_<double>(4, 4) <<
		-1.0, 0.0, 0.0, 0.0,
		0.0, -1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 3.0,
		0.0, 0.0, 0.0, 1.0);


	const std::string sign_quit = "q";//continue sign.
	const std::string sign_continue = "c";


public:
	std::string rootDir;
	std::string file_intrinsic_left;
	std::string file_intrinsic_right;
	std::string file_extrinsic_left;
	std::string file_extrinsic_right;
	Triangulation tri;

	Calibration(const std::string& rootDir)
		:rootDir(rootDir),
		file_intrinsic_left(rootDir + "/camera0_intrinsics.dat"),
		file_intrinsic_right(rootDir + "/camera1_intrinsics.dat"),
		file_extrinsic_left(rootDir + "/camera0_rot_trans.dat"),
		file_extrinsic_right(rootDir + "/camera1_rot_trans.dat"),
		tri(file_intrinsic_left, file_intrinsic_right, file_extrinsic_left, file_extrinsic_right)
	{
		std::cout << "construct a Eye to Hand calibration class" << std::endl;
	};

	~Calibration() {};

	/**
	* @brief main procedure for eye to hand calibration. 
	* 1. get data of images and TCP pose. 2.undistort images and save. 3. corner detection and show display to check whether we'll reverse the points. 4. 
	* @param[in] imgDir, csvDir image and csv directory.
	*/
	void main();

	/**
	* @brief read csv file and load data
	* @return {num_trials, (x,y,z,nx,ny,nz)}. TCP (Tool center position) pose. (nx,ny,nz) is a rotation vector.
	*/
	std::vector<std::vector<double>> readCSV(std::string& file_name);
	
	/**
	* @brief create homogeneous matrix from pose
	* @param[in] pose {x,y,z,nx,ny,nz}. (nx,ny,nz) is a rotation vector.
	* @return cv::Mat (4,4) [R T]
	*/
	cv::Mat createHomogeneousMatrix(std::vector<double>& pose);
};