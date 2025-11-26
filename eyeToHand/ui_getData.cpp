#include "ui_getData.h"

void UI_getData::main(std::vector<cv::Mat>& Hs_chess2camera, std::vector<cv::Mat>& Hs_tcp2base) {


    int counter=0;
    std::string bool_save, bool_continue;
    std::string file_path, name_file,file_left,file_right,file_csv;
    std::vector<double> joints_ur, tcp_ur;
    std::array<cv::Mat1b, 2> frames;
    int frameIndex;

    std::cout<<"start collecting data"<<std::endl;
    while (true) {
        //get current robot angles.
        joints_ur = urDI->getActualQ();

        std::cout << "$$$$$$$$$$ current joint angle=";
        for (double& joint : joints_ur)
            std::cout << joint << ",";
        std::cout << std::endl;

        //incrementals of each joint angle. 
        std::vector<double> jointValues(6, 0);
        //get incremental of each joint.
        adjustRobot(jointValues);

        // ----- move robot with moveJ()
        for (int j = 0; j < jointValues.size(); j++) {

            //crop moved angles.
            if (jointValues[j] < 0.0) {//negative value
                jointValues[j] = -std::min(std::abs(jointValues[j]), move_max);
            }
            else if (jointValues[j] > 0.0) {//negative value
                jointValues[j] = std::min(jointValues[j], move_max);
            }
            //add incremetal angle to current joints.
            joints_ur[j] += jointValues[j];
        }
        urCtrl->moveJ(joints_ur, 0.5, 0.5);//acceleration, velocity [rad/sec]
        urCtrl->stopJ();

        //hear whether save image and TCP pose.
        std::cout << "Will you save images and TCP pose?" << std::endl;
        std::cout << "type \'s\' if you wanna save, and otherwise type \'n\':";
        std::cin >> bool_save;

        //save image and extract 3D position.
        if (bool_save.compare(sign_save) == 0) {
            //---------- get an image from RealSense.
            bool bool_ok = false;
            cv::Mat img;
            // Initialize the aligned_depth_frame as an empty/default-constructed frame
            rs2::depth_frame* aligned_depth_frame = nullptr;


            while (!bool_ok) {
                if (!q_realsense_img.empty()) {//not emppty
                    std::pair<cv::Mat, rs2::depth_frame> frames = q_realsense_img.front();//extract data.
                    img = frames.first;
                    aligned_depth_frame = &(frames.second);//object>ptr
                    bool_ok = true;
                }
            }

            //------------ Detect corners.
            std::vector<cv::Point2f> corners;
            _corner_detector.getCorners(img, corners);

            // For a chessboard of size width x height, there should be width*height corners
            const int expected_num_corners = _corner_detector.width * _corner_detector.height;

            if (static_cast<int>(corners.size()) >= expected_num_corners) {
                // Prepare 3D object points for the chessboard in its coordinate system (z=0 plane)
                std::vector<cv::Point3f> objectPoints;
                for (int i = 0; i < _corner_detector.height; ++i) {
                    for (int j = 0; j < _corner_detector.width; ++j) {
                        objectPoints.emplace_back(j * _corner_detector.square_size, i * _corner_detector.square_size, 0.0f); // square_size is known or provided
                    }
                }

                // Camera parameters
                cv::Mat cameraMatrix, distCoeffs;
                // Assume _color_intrinsics is filled as in pyrealsense2/opencv conversion
                cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
                cameraMatrix.at<double>(0, 0) = _color_intrinsics.fx;  // Focal length in x (pixels)
                cameraMatrix.at<double>(1, 1) = _color_intrinsics.fy;  // Focal length in y (pixels)
                cameraMatrix.at<double>(0, 2) = _color_intrinsics.ppx; // Principal point x-coordinate (pixels)
                cameraMatrix.at<double>(1, 2) = _color_intrinsics.ppy; // Principal point y-coordinate (pixels)
                distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

                cv::Mat rvec, tvec;
                bool found = cv::solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);

                if (found) {
                    // Convert rvec and tvec to a homogeneous transformation matrix
                    cv::Mat R;
                    cv::Rodrigues(rvec, R);
                    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
                    for (int row = 0; row < 3; ++row) {
                        for (int col = 0; col < 3; ++col) {
                            T.at<float>(row, col) = static_cast<float>(R.at<double>(row, col));
                        }
                        T.at<float>(row, 3) = static_cast<float>(tvec.at<double>(row)) * 1000.0f; // [m] to [mm]
                    }
                    Hs_chess2camera.push_back(T.clone());
                    std::cout << "board pose estimated by solvePnP" << std::endl;

                    //-------- robot's end-effector pose.
                    std::vector<double> eef_pose = urDI->getActualTCPPose();
                    eef_pose[0] *= 1000.0;eef_pose[1] *= 1000.0;eef_pose[2] *= 1000.0;//mm
                    cv::Mat H_eef = createHomogeneousMatrix(eef_pose);
                    Hs_tcp2base.push_back(H_eef.clone());

                    counter++;
                }
            }
                    else {
                        std::cout << "Failed to compute chessboard 3D pose: insufficient or invalid corner depths." << std::endl;
                    }
                }
                else {
                    std::cout << "Invalid chessboard width or insufficient number of corners detected." << std::endl;
                }
            }
            else {
                std::cout << "Failed to detect sufficient chessboard corners." << std::endl;
            }
        }

        //continue
        std::cout << "Will you continue?" << std::endl;
        std::cout << "type \'c\' if you wanna continue, and if you wanna quit, type \'q\':";
        std::cin >> bool_continue;
        //save data
        if (bool_continue.compare(sign_quit) == 0) {
            std::cout << "Will you really stop?" << std::endl;
            std::cout << "type \'q\' if you wanna quit, and otherwise type \'c\':";
            std::cin >> bool_continue;
            if (bool_continue.compare(sign_quit) == 0)
                break;
        }
    }
    std::cout << "finish getting data" << std::endl;
}

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
    if (norm_rvec > PI) {//0<=theta<=PI
        double theta = std::fmod(norm_rvec, 2.0 * PI);
        if (theta <= PI) {
            rvec = theta / norm_rvec * rvec;
        }
        else if (theta > PI) {//PI<theta<=2*PI
            theta = theta - 2.0 * PI;
            rvec = theta / norm_rvec * rvec;
        }
    }

    // Convert rotation vector to rotation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Create translation vector
    cv::Mat t = (cv::Mat_<double>(3, 1) << pose[0], pose[1], pose[2]);
    
    //create homogeneous matrix
    cv::Mat H_matrix;
    cv::Mat oneVec = cv::Mat1d::zeros(1, 4);
    oneVec.at<double>(0, 3) = 1;
    cv::hconcat(R, t, H_matrix);
    cv::vconcat(H_matrix, oneVec, H_matrix);

    return H_matrix;
}