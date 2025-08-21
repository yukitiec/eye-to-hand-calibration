// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
//

#pragma once
#pragma comment(lib, "rtde.lib")

#include "targetver.h"

//utility
#include <iostream>
#include <string>
#include <chrono>
#include <atomic>
#include <thread>
#include <vector>
#include <array>
#include <queue>
#include <mutex>
#include <chrono>
#include <fstream>
#include <sstream> // For stringstream
#include <ctime>
#include <direct.h>
#include <sys/stat.h>
#include <cmath>
#include <filesystem>
#include <iomanip>

//Matrix
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

//boost : about queue
#include <boost/asio.hpp>
#include <boost/lockfree/spsc_queue.hpp>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/optflow/rlofflow.hpp>
#include <opencv2/tracking.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/core/hal/hal.hpp"
#include "opencv2/core/ocl.hpp"
#include <opencv2/tracking/tracking_legacy.hpp>

#include "ximea.h"

//existed header files
#include "type_definition.h"
#include "communicate_dspace.h"
#include "display_info.h"
#include <iso646.h> 
#include "progressbar.hpp"

//UR
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

//original header file
//#include "skeleton.h" //skeleton.h
#include "global_parameters.h" //global parameters
//#include "yolo_batch.h" // YOLO-pose skeleton detection
//#include "optflow.h" //optical flow tracking
//#include "triangulation.h" //triangulate points 
#include "utility.h" //utility function


#define SYNC_CAMERAS (1) //1:同期，0:非同期
