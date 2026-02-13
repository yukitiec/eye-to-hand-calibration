#include "stdafx.h"
#include "global_parameters.h"

//in pixel selection
const int N_NEIGHBOR = 5;//3 for 20mm*20mm, 7 for 45mm*45mm
const int _IMG_WIDTH = 1280;//Ximea : 512, RealSense L515:640, D435 : 1280 for depth, 1920
const int _IMG_HEIGHT = 720;//Ximea : 512, RealSense L515:480, D435 : 720 for depth, 1080 for RGB

/* constant valude definition */
rs2_intrinsics _color_intrinsics{};
std::queue<std::pair<cv::Mat, rs2::depth_frame>> q_realsense_img;

extern const std::string file_ur = "ur.csv";

//UR setting
extern const std::string URIP = "10.0.4.10";//10.0.3.7 for AMIGA, 10.0.4.10 for AMIGO// "169.254.32.90";
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