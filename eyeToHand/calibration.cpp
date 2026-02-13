#include "calibration.h"

void Calibration::main() {
    
    int type_process,width, height, size;
    std::string bool_mode,bool_continue;

    std::this_thread::sleep_for(std::chrono::milliseconds(7000));

    while (true) {
        //continue
        std::cout << "Hello! Will you continue?" << std::endl;
        std::cout << "type \'c\' if you wanna continue, and if you wanna stop, type \'q\':";
        std::cin >> bool_continue;
        //save data
        if (bool_continue.compare(sign_quit) == 0) {
            std::cout << "Will you really stop?" << std::endl;
            std::cout << "type \'q\' if you wanna quit, and otherwise type \'c\':";
            std::cin >> bool_continue;
            if (bool_continue.compare(sign_quit) == 0)
                break;
        }

        //get running types
        std::cout << ":: Which procedure will you do? input an integer.\n 1: get data for eye to hand calibration.\n 2: evaluate sterep camera accuracy. check depth. \n";
        std::cin >> type_process;
        std::cout << "You chose Eye-to-Hand calibration. If it's okay type \'c\'. If you want to change mode, type \'q\'" << std::endl;
        std::cin >> bool_mode;
        //save data
        if (bool_mode.compare(sign_continue) == 0) {
            //collect data for eye to hand calibration.
            //------- move robot to initial position
            //for tidyup
            std::vector<double> config_init{ //
                    PI / 180 * -12.62,//36.62,//12.16,
                    PI / 180 * -132.0,//-34.7,//(-42.68), 
                    PI / 180 * -35.0,//-123.39,//(-139.34),
                    PI / 180 * -57.0,//-20.42,//(-0.60),
                    PI / 180 * 214,//124.72,//110.4,
                    PI / 180 * -100.0//-100.41//(-88.87)
            };

            //for catching
            //std::vector<double> config_init{ //
            //        PI / 180 * -4.5,//36.62,//12.16,
            //        PI / 180 * -31.0,//-34.7,//(-42.68), 
            //        PI / 180 * -105.0,//-123.39,//(-139.34),
            //        PI / 180 * -21.0,//-20.42,//(-0.60),
            //        PI / 180 * 91.0,//124.72,//110.4,
            //        PI / 180 * -103.0//-100.41//(-88.87)
            //};
            urCtrl->moveJ(config_init, 0.5, 0.5);//speed, acceleration
				

			//--- Collect data & save 3D pose pairs.
			std::vector<cv::Mat> Hs_chess2camera; 
			std::vector<cv::Mat> Hs_tcp2base;
			ui.main(Hs_chess2camera, Hs_tcp2base, type_process);
            q_end_process.push(true);

            // ---- Eye-to-Hand Calibration Main Block ----
            // Note:
            // Hs_chess2camera: 4x4 homogeneous pose of the calibration target (chessboard/board) in the *camera* frame.
            // Hs_tcp2base: 4x4 homogeneous pose of the end-effector (tcp) in the *robot base* frame.
            // In cv::calibrateHandEye, the classical convention is:
            //   A : end-effector in base (base->tcp), which here is (Hs_tcp2base)^-1
            //   B : calibration target in camera (chess2camera), i.e. Hs_chess2camera
            // Compose 4x4 homogeneous camera-to-base transformation matrix
            size_t num_pose = Hs_chess2camera.size();
            cv::Mat H_cam2base = cv::Mat::eye(4, 4, CV_64F);
            if (type_process == 1) {
                
                if (num_pose != Hs_tcp2base.size()) {
                    std::cerr << "[Calibration] Number of board and tcp poses does not match!\n";
                    return;
                }

                std::vector<cv::Mat> R_gripper2base(num_pose), t_gripper2base(num_pose);
                std::vector<cv::Mat> R_target2cam(num_pose), t_target2cam(num_pose);

                for (size_t idx = 0; idx < num_pose; ++idx) {
                    // OpenCVHandEye expects:
                    // R_gripper2base, t_gripper2base: transformation from gripper (tcp) to base (i.e., base->tcp)
                    // R_target2cam, t_target2cam: transformation from calibration target to camera (i.e., camera->chess)

                    // The provided Hs_tcp2base is tcp->base, so invert to base->tcp
                    cv::Mat H_base2tcp;
                    cv::invert(Hs_tcp2base[idx], H_base2tcp, cv::DECOMP_SVD);
                    if (H_base2tcp.type() != CV_64F)
                        H_base2tcp.convertTo(H_base2tcp, CV_64F);

                    R_gripper2base[idx] = H_base2tcp(cv::Rect(0, 0, 3, 3)).clone();
                    t_gripper2base[idx] = H_base2tcp(cv::Rect(3, 0, 1, 3)).clone();
                    if (R_gripper2base[idx].type() != CV_64F)
                        R_gripper2base[idx].convertTo(R_gripper2base[idx], CV_64F);
                    if (t_gripper2base[idx].type() != CV_64F)
                        t_gripper2base[idx].convertTo(t_gripper2base[idx], CV_64F);

                    // Hs_chess2camera is chess (board) in camera frame. We need target->camera, NOT camera->target.
                    // But OpenCV uses 'target-to-camera', so Hs_chess2camera is fine.
                    cv::Mat H_target2cam = Hs_chess2camera[idx];
                    if (H_target2cam.type() != CV_64F)
                        H_target2cam.convertTo(H_target2cam, CV_64F);

                    R_target2cam[idx] = H_target2cam(cv::Rect(0, 0, 3, 3)).clone();
                    t_target2cam[idx] = H_target2cam(cv::Rect(3, 0, 1, 3)).clone();
                    if (R_target2cam[idx].type() != CV_64F)
                        R_target2cam[idx].convertTo(R_target2cam[idx], CV_64F);
                    if (t_target2cam[idx].type() != CV_64F)
                        t_target2cam[idx].convertTo(t_target2cam[idx], CV_64F);
                }

                cv::Mat R_cam2base, t_cam2base;
                // This solves for (R,T): gripper2base * X = X * target2cam -> X = cam2base
                cv::calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    R_cam2base, t_cam2base,
                    cv::CALIB_HAND_EYE_TSAI
                );

                R_cam2base.copyTo(H_cam2base(cv::Range(0, 3), cv::Range(0, 3)));
                t_cam2base.copyTo(H_cam2base(cv::Range(0, 3), cv::Range(3, 4)));

                // Save transformation
                std::ofstream file_transform("transform_camera2base.csv");
                if (!file_transform.is_open()) {
                    std::cerr << "Error opening file for writing." << std::endl;
                }
                else {
                    for (int row = 0; row < H_cam2base.rows; ++row) {
                        for (int col = 0; col < H_cam2base.cols; ++col) {
                            file_transform << H_cam2base.at<double>(row, col);
                            if (col < H_cam2base.cols - 1)
                                file_transform << ",";
                        }
                        file_transform << "\n";
                    }
                    file_transform.close();
                }
            }
            else {
                // Load transformation from CSV
                std::ifstream file_transform("transform_camera2base.csv");
                if (!file_transform.is_open()) {
                    std::cerr << "Error opening file for reading." << std::endl;
                } else {
                    std::string line;
                    int row = 0;
                    while (std::getline(file_transform, line) && row < H_cam2base.rows) {
                        std::stringstream ss(line);
                        std::string value;
                        int col = 0;
                        while (std::getline(ss, value, ',') && col < H_cam2base.cols) {
                            double v = std::stod(value);
                            H_cam2base.at<double>(row, col) = v;
                            ++col;
                        }
                        ++row;
                    }
                    file_transform.close();
                }
            }

            // ------ Evaluation of the calibration ------
            // For each frame, check camera estimate in base frame:
            // P_base_est = H_cam2base * H_chess2camera
            // Compare translation against true tcp pose (Hs_tcp2base)
            for (size_t i = 0; i < num_pose; ++i) {
                cv::Mat H_cam2base_64F = H_cam2base;
                cv::Mat H_chess2camera_64F = Hs_chess2camera[i];
                if (H_chess2camera_64F.type() != CV_64F)
                    H_chess2camera_64F.convertTo(H_chess2camera_64F, CV_64F);

                cv::Mat H_est_base = H_cam2base_64F * H_chess2camera_64F;

                cv::Mat t_est = H_est_base(cv::Rect(3,0,1,3)).clone();
                cv::Mat t_true = Hs_tcp2base[i](cv::Rect(3,0,1,3)).clone();
                if (t_true.type() != CV_64F)
                    t_true.convertTo(t_true, CV_64F);

                cv::Mat diff = t_est - t_true;
                double error = cv::norm(diff);

                std::cout << "--- Frame " << i << " ---\n";
                std::cout << "Estimated end-effector (from camera): [" << t_est.at<double>(0) << ", "
                            << t_est.at<double>(1) << ", " << t_est.at<double>(2) << "]\n";
                std::cout << "Actual end-effector (from robot):      [" << t_true.at<double>(0) << ", "
                            << t_true.at<double>(1) << ", " << t_true.at<double>(2) << "]\n";
                std::cout << "Position error: " << error << " mm\n";
                std::cout << "--------------------------" << std::endl;
            }
        }

    }
    

    std::cout << "load camera intrinsic paramters. wait a few seconds. ....." << std::endl;
    //load camera intrinsic parameters.
    //K, dist, Rotation, Translation.

    //get chessboard information. height, width, size.
    /*std::cout << ":: Chessboard information :: \n width=";
    std::cin >> width;
    std::cout << "height=";
    std::cin >> height;
    std::cout << "size=";
    std::cin >> size;
    std::cout << std::endl;*/

    //for each data
    //1.undistort images and save. 
    //2.corner detection and show display to check whether we'll reverse the points.
    //3.calculate H_Camera2Chess with cv::solvePnP -> rotation vector and translation.
    //save in the storage
    

    //cv::calibrateHandEye() -> get transformation matrix.

}

std::vector<std::vector<double>> Calibration::readCSV(std::string& file_name) {
    /**
    * @brief read CSV file and save data into a vector.
    * @param[in] file path
    */

    // Create a 2D vector to store the CSV data
    std::vector<std::vector<double>> data;

    // Open the CSV file
    std::ifstream file(file_name);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_name << std::endl;
        return std::vector<std::vector<double>>{};
    }

    std::string line;

    // Read the file line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        // Split the line by commas and store the values in the row vector
        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value)); // Convert the string to double
        }

        // Add the row vector to the data vector
        data.push_back(row);
    }

    // Close the file
    file.close();

    // Output the data to verify
    for (const auto& row : data) {
        for (const auto& value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    return data;
}

cv::Mat Calibration::createHomogeneousMatrix(std::vector<double>& pose) {
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