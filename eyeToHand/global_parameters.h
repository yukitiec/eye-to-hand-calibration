#pragma once

#ifndef GLOBAL_PARAMETERS_H
#define GLOBAL_PARAMETERS_H

#include "stdafx.h"

extern const int N_NEIGHBOR;
extern rs2_intrinsics _color_intrinsics;
extern std::queue<std::pair<cv::Mat, rs2::depth_frame>> q_realsense_img;
extern const int _IMG_WIDTH;//Ximea : 512, RealSense L515:640, D435 : 1280 for depth, 1920
extern const int _IMG_HEIGHT;//Ximea : 512, RealSense L515:480, D435 : 720 for depth, 1080 for RGB
extern std::mutex mtxRobot;

/* from joints to robot control */
extern std::queue<std::vector<std::vector<std::vector<int>>>> queueJointsPositions;
/* notify danger */
extern std::queue<bool> queueDanger;


//UR setting
using namespace ur_rtde;

extern const std::string URIP;
extern std::unique_ptr<RTDEControlInterface> urCtrl;
extern std::unique_ptr<RTDEIOInterface> urDO;
extern std::unique_ptr<RTDEReceiveInterface> urDI;

extern std::queue<bool> q_startTracking; //start tracking
extern std::queue<bool> q_endTracking,q_end_process; //end tracking

//mutex for img
extern std::mutex mtx_img;
#endif