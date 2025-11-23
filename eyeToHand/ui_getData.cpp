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
                //-------- Get 3 points to compute the board's 3D pose.

                // Ensure the indices 0, 1, and width are valid
                if (_corner_detector.width >= 2 && _corner_detector.width < expected_num_corners) {
                    std::cout << "1" << std::endl;

                    cv::Point2f p_origin = corners[0];
                    cv::Point2f p_x = corners[1];
                    cv::Point2f p_y = corners[_corner_detector.width];

                    std::vector<cv::Point2f> ps_2d{ p_origin, p_x, p_y };
                    std::vector<cv::Point3f> ps_3d;

                    //------- Compute 3D position of each corner
                    bool bool_valid = true;
                    for (const auto& p : ps_2d) {
                        bool valid = (p.x >= 0 && p.x < _color_intrinsics.width && p.y >= 0 && p.y < _color_intrinsics.height);
                        float depth_meters = 0.0f;
                        if (p.x >= 0 && p.y >= 0 && valid) {
                            try {
                                depth_meters = aligned_depth_frame->get_distance(static_cast<int>(p.x), static_cast<int>(p.y));
                                float pixel[2] = { p.x, p.y };
                                float point_3d[3];
                                rs2_deproject_pixel_to_point(point_3d, &_color_intrinsics, pixel, depth_meters);
                                ps_3d.emplace_back(point_3d[0], point_3d[1], point_3d[2]);
                            }
                            catch (const rs2::error& e) {
                                // Print the prompt error only on first exception for demo
                                static bool error_printed = false;
                                if (!error_printed) {
                                    std::cerr << "RealSense error: " << e.what() << std::endl;
                                    error_printed = true;
                                }
                                depth_meters = 0.0f;
                                bool_valid = false;
                            }
                        }
                        else {
                            bool_valid = false;
                        }
                    }
                    std::cout << "2" << std::endl;

                    if (bool_valid && ps_3d.size() == 3) {
                        //---------- Calculate the board's pose axes
                        cv::Point3f Porigin = ps_3d[0];
                        cv::Point3f Px = ps_3d[1];
                        cv::Point3f Py = ps_3d[2];

                        cv::Point3f ex = Px - Porigin;
                        ex /= cv::norm(ex);
                        cv::Point3f ey = Py - Porigin;
                        ey /= cv::norm(ey);
                        cv::Point3f ez = ex.cross(ey);
                        ez /= cv::norm(ez);

                        //---------- Build rotation matrix as cv::Mat (3x3)
                        cv::Mat R(3, 3, CV_32F);
                        R.at<float>(0, 0) = ex.x; R.at<float>(0, 1) = ey.x; R.at<float>(0, 2) = ez.x;
                        R.at<float>(1, 0) = ex.y; R.at<float>(1, 1) = ey.y; R.at<float>(1, 2) = ez.y;
                        R.at<float>(2, 0) = ex.z; R.at<float>(2, 1) = ey.z; R.at<float>(2, 2) = ez.z;

                        //---------- Compose 4x4 homogeneous transformation matrix
                        cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
                        R.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
                        T.at<float>(0, 3) = Porigin.x*1000.0;//mm
                        T.at<float>(1, 3) = Porigin.y*1000.0;
                        T.at<float>(2, 3) = Porigin.z*1000.0;
                        Hs_chess2camera.push_back(T.clone());
                        std::cout << "3" << std::endl;

                        //-------- robot's end-effector pose.
                        std::vector<double> eef_pose = urDI->getActualTCPPose();
                        eef_pose[0] *= 1000.0;eef_pose[1] *= 1000.0;eef_pose[2] *= 1000.0;//mm
                        cv::Mat H_eef = createHomogeneousMatrix(eef_pose);
                        Hs_tcp2base.push_back(H_eef.clone());

                        counter++;
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