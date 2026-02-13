#include "ui_getData.h"
#include <random>

void UI_getData::main(std::vector<cv::Mat>& Hs_chess2camera, std::vector<cv::Mat>& Hs_tcp2base, const int mode_idx) {

    //file 
    const std::string poseCsv = "saved_joints.csv";
    if (mode_idx == 1) {//Eye-to-hand calibration
        auto savedPoses = loadPosesCSV(poseCsv);
        if (!savedPoses.empty()) {
            replaySavedPoses(savedPoses, urCtrl, Hs_chess2camera, Hs_tcp2base);
            // Option 2: pass a raw pointer
            // ui.replaySavedPoses(savedPoses, urCtrl.get(), Hs_chess2camera, Hs_tcp2base);
        }
    }
    else {//evaluaiton.
        const std::string poseCsv_eval = "saved_joints_eval.csv";
        auto savedPoses = loadPosesCSV(poseCsv_eval);
        if (!savedPoses.empty()) {
            replaySavedPoses(savedPoses, urCtrl, Hs_chess2camera, Hs_tcp2base);
            // Option 2: pass a raw pointer
            // ui.replaySavedPoses(savedPoses, urCtrl.get(), Hs_chess2camera, Hs_tcp2base);
        }
        return;//
    }



    int counter = 0;
    std::string bool_save, bool_continue;
    std::string file_path, name_file, file_left, file_right, file_csv;
    std::vector<double> joints_ur, tcp_ur;
    std::array<cv::Mat1b, 2> frames;
    int frameIndex;

    // ---- Buffers for calibration data ----

    const std::string sign_save = "s";
    const std::string sign_quit = "q";

    // ----------------- Helper: capture one calibration sample -----------------



    // 0: manual, 1: automatic random data collection
    int collect_mode = 0;
    while (true) {
        //get current robot angles.
        std::vector<double> joints_ur = urDI->getActualQ();

        std::cout << "$$$$$$$$$$ current joint angle=";
        for (double& joint : joints_ur)
            std::cout << joint << ",";
        std::cout << std::endl;

        collect_mode = 0;
        std::cout << "Select data collection mode (0: manual, 1: automatic random): ";
        std::cin >> collect_mode;

        // ---- Parameters for automatic random motion ----
        std::vector<double> initial_joints = urDI->getActualQ(); // or joints_ivpf

        // ======================== AUTOMATIC MODE ============================
        if (collect_mode == 1) {
            std::cout << "start collecting data (automatic mode)" << std::endl;
            // ---- Random generator for automatic mode ----
            int num_auto_samples = 7;       // 10 data collects
            double range_moveJ = 0.15;       // random range per joint [rad]
            const double move_max = 0.1;           // manual small step limit [rad]
            
            std::cout << "num_samples: ";
            std::cin >> num_auto_samples;
            std::cout << "Random joint range ";
            std::cin >> range_moveJ;
            
            std::mt19937 rng(std::random_device{}());
            std::uniform_real_distribution<double> dist(-range_moveJ, range_moveJ);

            for (int k = 0; k < num_auto_samples; ++k) {
                std::cout << "Automatic sample " << (k + 1) << " / " << num_auto_samples << std::endl;

                // 1) Generate random joint target around initial_joints
                std::vector<double> target_joints = initial_joints;
                for (size_t i = 0; i < target_joints.size(); ++i) {
                    double delta = dist(rng); // in [-range_moveJ, range_moveJ]
                    target_joints[i] += delta;
                }

                // 2) Move robot
                urCtrl->moveJ(target_joints, 0.5, 0.5); // you can adjust acc/vel
                std::this_thread::sleep_for(std::chrono::seconds(4));

                // 3) Capture one calibration sample (with corner detection & solvePnP)
                bool ok = capture_calibration_sample(k + 1,true, Hs_chess2camera, Hs_tcp2base);
                if (ok) {
                    // Save only when capture succeeded
                    appendPoseCSV(poseCsv, target_joints);
                }
                else {
                    std::cout << "Warning: sample " << (k + 1) << " invalid (no chessboard / pose)." << std::endl;
                }
            }

            std::cout << "finish automatic data collection" << std::endl;
        }
        // ======================== MANUAL MODE ===============================
        else {
            std::cout << "start collecting data (manual mode)" << std::endl;

            int sample_id = 0;


            //incrementals of each joint angle. 
            std::vector<double> jointValues(6, 0.0);
            //get incremental of each joint (e.g., keyboard input, joystick, etc.)
            adjustRobot(jointValues);

            // ----- move robot with moveJ()
            for (int j = 0; j < (int)jointValues.size(); j++) {
                // crop moved angles
                if (jointValues[j] < 0.0) {
                    jointValues[j] = -std::min(std::abs(jointValues[j]), move_max);
                }
                else if (jointValues[j] > 0.0) {
                    jointValues[j] = std::min(jointValues[j], move_max);
                }
                // add incremental angle to current joints
                joints_ur[j] += jointValues[j];
            }
            urCtrl->moveJ(joints_ur, 0.5, 0.5); // acceleration, velocity [rad/sec]
            urCtrl->stopJ();

            // ask whether to save image and TCP pose
            std::cout << "Will you save images and TCP pose?" << std::endl;
            std::cout << "type 's' if you wanna save, and otherwise type 'n': ";
            std::cin >> bool_save;

            if (bool_save.compare(sign_save) == 0) {
                ++sample_id;
                bool ok = capture_calibration_sample(sample_id,false, Hs_chess2camera, Hs_tcp2base);
                if (ok) {
                    // Save only when capture succeeded
                    appendPoseCSV(poseCsv, joints_ur);
                }
                else {
                    std::cout << "Failed to capture valid sample." << std::endl;
                }
            }
        }
        //continue?
        std::cout << "Will you continue?" << std::endl;
        std::cout << "type 'c' if you wanna continue, and if you wanna quit, type 'q': ";
        std::cin >> bool_continue;

        if (bool_continue.compare(sign_quit) == 0) {
            std::cout << "Will you really stop?" << std::endl;
            std::cout << "type 'q' if you wanna quit, and otherwise type 'c': ";
            std::cin >> bool_continue;
            if (bool_continue.compare(sign_quit) == 0)
                break;
        }

        std::cout << "finish getting data (manual mode)" << std::endl;
    }

    // ======================= SAVE MATRICES TO CSV =======================
    // Save Hs_chess2camera to camera.csv and Hs_tcp2base to robot.csv as (N_data, 16) CSVs
    {
        std::ofstream camera_file("camera.csv"), robot_file("robot.csv");
        if (!camera_file.is_open()) {
            std::cerr << "Failed to open camera.csv for writing." << std::endl;
        }
        if (!robot_file.is_open()) {
            std::cerr << "Failed to open robot.csv for writing." << std::endl;
        }

        auto write_matrix_flattened = [](std::ofstream& file, const cv::Mat& mat) {
            // mat should be 4x4, type CV_32F
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    file << mat.at<float>(row, col);
                    if (!(row == 3 && col == 3)) file << ",";
                }
            }
            file << "\n";
            };

        if (camera_file.is_open()) {
            for (const auto& mat : Hs_chess2camera) {
                write_matrix_flattened(camera_file, mat);
            }
            camera_file.close();
        }

        if (robot_file.is_open()) {
            for (const auto& mat : Hs_tcp2base) {
                write_matrix_flattened(robot_file, mat);
            }
            robot_file.close();
        }
    }
	
}

bool UI_getData::capture_calibration_sample(int sample_id, bool is_automatic, std::vector<cv::Mat>& Hs_chess2camera, std::vector<cv::Mat>& Hs_tcp2base)
{
    //---------- get an image from RealSense.
    bool bool_ok = false;
    cv::Mat img;
    rs2::depth_frame* aligned_depth_frame = nullptr;

    // Wait until we get one frame from queue
    while (!bool_ok) {
        if (!q_realsense_img.empty()) { // not empty
            std::pair<cv::Mat, rs2::depth_frame> frames = q_realsense_img.front(); // extract data
            img = frames.first;
            aligned_depth_frame = &(frames.second); // object -> ptr
            bool_ok = true;
        }
        // optional: std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    //------------ Detect corners.
    std::vector<cv::Point2f> corners;
    _corner_detector.getCorners(img, corners, is_automatic);

    // For a chessboard of size width x height, there should be width*height corners
    const int expected_num_corners = _corner_detector.width * _corner_detector.height;

    if (static_cast<int>(corners.size()) >= expected_num_corners) {
        // Prepare 3D object points for the chessboard in its coordinate system (z=0 plane)
        std::vector<cv::Point3f> objectPoints;
        objectPoints.reserve(expected_num_corners);
        for (int i = 0; i < _corner_detector.height; ++i) {
            for (int j = 0; j < _corner_detector.width; ++j) {
                objectPoints.emplace_back(
                    j * _corner_detector.size,
                    i * _corner_detector.size,
                    0.0f
                );
            }
        }

        // Camera parameters
        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = _color_intrinsics.fx;  // Focal length in x
        cameraMatrix.at<double>(1, 1) = _color_intrinsics.fy;  // Focal length in y
        cameraMatrix.at<double>(0, 2) = _color_intrinsics.ppx; // Principal point x
        cameraMatrix.at<double>(1, 2) = _color_intrinsics.ppy; // Principal point y

        cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

        cv::Mat rvec, tvec;
        bool found = cv::solvePnP(objectPoints, corners,
            cameraMatrix, distCoeffs,
            rvec, tvec);

        if (found) {
            // Convert rvec and tvec to a homogeneous transformation matrix
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    T.at<float>(row, col) = static_cast<float>(R.at<double>(row, col));
                }
                // NOTE: tvec unit depends on your calibration pipeline; comment says [mm]
                T.at<float>(row, 3) = static_cast<float>(tvec.at<double>(row));
            }
            Hs_chess2camera.push_back(T.clone());
            std::cout << "[Sample " << sample_id << "] board pose estimated by solvePnP" << std::endl;

            //-------- robot's end-effector pose.
            std::vector<double> eef_pose = urDI->getActualTCPPose();
            // Convert to mm if needed
            eef_pose[0] *= 1000.0;
            eef_pose[1] *= 1000.0;
            eef_pose[2] *= 1000.0;
            cv::Mat H_eef = createHomogeneousMatrix(eef_pose);
            std::cout << "H_eef=" << H_eef << std::endl;
            Hs_tcp2base.push_back(H_eef.clone());

            return true;
        }
        else {
            std::cout << "[Sample " << sample_id << "] Failed to compute chessboard 3D pose: solvePnP failed." << std::endl;
            return false;
        }
    }
    else {
        std::cout << "[Sample " << sample_id << "] Invalid chessboard width or insufficient number of corners detected." << std::endl;
        return false;
    }
};

void UI_getData::adjustRobot(std::vector<double>& jointValues) {

    std::cout << "Please input values for the following joints:\n";
    for (size_t i = 0; i < jointNames.size(); ++i) {
        std::cout << "Enter incremental angle for " << jointNames[i] << " [rad] : ";
        while (!(std::cin >> jointValues[i])) {
            std::cin.clear(); // clear the error flag
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard invalid input
            std::cout << "Invalid input. Please enter a numerical value for " << jointNames[i] << ": ";
        }
    }
    std::cout << "\nYou entered the following values:\n";
    for (size_t i = 0; i < jointNames.size(); ++i) {
        std::cout << jointNames[i] << ": " << jointValues[i] << ",";
    }
    std::cout << std::endl;
}

void UI_getData::makeDir(std::filesystem::path& dirPath) {
    /**
    * @brief make a directory.
    */

    try {
        if (std::filesystem::create_directory(dirPath)) {
            std::cout << "Directory created successfully: " << dirPath << std::endl;
        }
        else {
            std::cout << "Directory already exists or could not be created: " << dirPath << std::endl;
        }
    }
    catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    }
}

cv::Mat UI_getData::createHomogeneousMatrix(std::vector<double>& pose) {
    /**
    * @brief create homogeneous transformation matrix from std::vector<double> which is (x,y,z,nx,ny,nz)
    */
    const double PI = CV_PI;
    // Create rotation vector
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
    
    ////normalize between 0 and pi
    double norm_rvec = cv::norm(rvec);
    //if (norm_rvec > PI) {//0<=theta<=PI
    //    double theta = std::fmod(norm_rvec, 2.0 * PI);
    //    if (theta <= PI) {
    //        rvec = theta / norm_rvec * rvec;
    //    }
    //    else if (theta > PI) {//PI<theta<=2*PI
    //        theta = theta - 2.0 * PI;
    //        rvec = theta / norm_rvec * rvec;
    //    }
    //}

    // Convert rotation vector to rotation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Create translation vector
    cv::Mat t = (cv::Mat_<double>(3, 1) << pose[0], pose[1], pose[2]);
    
    //create homogeneous matrix
    cv::Mat H_matrix = cv::Mat::eye(4, 4, CV_32F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            H_matrix.at<float>(row, col) = static_cast<float>(R.at<double>(row, col));
        }
        // NOTE: tvec unit depends on your calibration pipeline; comment says [mm]
        H_matrix.at<float>(row, 3) = static_cast<float>(t.at<double>(row));
    }
    //cv::Mat oneVec = cv::Mat1d::zeros(1, 4);
    //oneVec.at<double>(0, 3) = 1;
    //cv::hconcat(R, t, H_matrix);
    //cv::vconcat(H_matrix, oneVec, H_matrix);

    return H_matrix;
}

void UI_getData::appendPoseCSV(const std::string& path, const std::vector<double>& joints) {
    std::ifstream ifs(path);
    bool hasFile = ifs.good();
    ifs.close();

    std::ofstream ofs(path, std::ios::app);
    if (!hasFile) {
        ofs << "j1,j2,j3,j4,j5,j6\n";
    }
    for (size_t i = 0; i < joints.size(); ++i) {
        ofs << joints[i];
        if (i + 1 < joints.size()) ofs << ",";
    }
    ofs << "\n";
}

// Load all 6-DOF joint poses from CSV
std::vector<std::vector<double>> UI_getData::loadPosesCSV(const std::string& path) {
    std::vector<std::vector<double>> poses;
    std::ifstream ifs(path);
    if (!ifs) return poses;

    std::string line;
    bool isHeader = true;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        if (isHeader) { isHeader = false; continue; } // skip header

        std::vector<double> pose;
        std::stringstream ss(line);
        std::string item;
        while (std::getline(ss, item, ',')) {
            try { pose.push_back(std::stod(item)); }
            catch (...) { pose.clear(); break; }
        }
        if (pose.size() == 6) poses.push_back(pose);
    }
    return poses;
}

void UI_getData::replaySavedPoses(const std::vector<std::vector<double>>& poses,
    ur_rtde::RTDEControlInterface* urCtrl, std::vector<cv::Mat>& Hs_chess2camera, std::vector<cv::Mat>& Hs_tcp2base) {
    if (!urCtrl) {
        std::cerr << "RTDEControlInterface pointer is null; cannot replay poses.\n";
        return;
    }

    std::cout << "Replaying " << poses.size() << " saved poses...\n Wait for 15 seconds ... ";
    std::this_thread::sleep_for(std::chrono::seconds(15));
    for (size_t i = 0; i < poses.size(); ++i) {
        std::cout << "Moving to saved pose " << (i + 1) << " / " << poses.size() << "\n";
        urCtrl->moveJ(poses[i], 0.5, 0.5); // acc, vel (adjust to your API order if needed)
        std::this_thread::sleep_for(std::chrono::seconds(5));

        bool ok = this->capture_calibration_sample(static_cast<int>(i + 1), true, Hs_chess2camera, Hs_tcp2base);
        if (!ok) {
            std::cout << "Warning: replay sample " << (i + 1)
                << " invalid (no chessboard / pose)." << std::endl;
        }
    }
    std::cout << "Replay finished.\n";
}