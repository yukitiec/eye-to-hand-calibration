#include "stdafx.h"
#include "global_parameters.h"


/* constant valude definition */
rs2_intrinsics _color_intrinsics{};
std::queue<std::pair<cv::Mat, rs2::depth_frame>> q_realsense_img;

extern const std::string file_ur = "ur.csv";

//UR setting
extern const std::string URIP = "169.254.32.90";
std::unique_ptr<RTDEControlInterface> urCtrl = std::make_unique<RTDEControlInterface>(URIP);
//RTDEControlInterface urCtrl(URIP);
std::unique_ptr<RTDEIOInterface> urDO = std::make_unique<RTDEIOInterface>(URIP);
std::unique_ptr<RTDEReceiveInterface> urDI = std::make_unique<RTDEReceiveInterface>(URIP);

std::mutex mtxRobot;
/* from joints to robot control */
std::queue<std::vector<std::vector<std::vector<int>>>> queueJointsPositions;
/* notify danger */
std::queue<bool> queueDanger;

//queue
std::queue<bool> q_startOptflow; //start optical flow
std::queue<bool> q_startTracking; //start tracking
std::queue<bool> q_endTracking, q_end_process; //end tracking
//mutex for img
std::mutex mtx_img;