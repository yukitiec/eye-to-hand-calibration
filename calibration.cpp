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
		if (type_process == 1) {
			std::cout << "You chose Eye-to-Hand calibration. If it's okay type \'c\'. If you want to change mode, type \'q\'" << std::endl;
			std::cin >> bool_mode;
			//save data
			if (bool_mode.compare(sign_continue) == 0) {
				//collect data for eye to hand calibration.
				//move robot to initial position
				std::vector<double> config_init{2.728,-2.716,0.8845,-1.16,-1.20,0.004};
				urCtrl->moveJ(config_init, 0.5, 0.5);//speed, acceleration
				ui.main();
				q_end_process.push(true);
				//calculate transformation matrix from camera frame to the robot base frame.
				//construct a chess2camera instance.
				Chess2camera chess2camera(rootDir);//rootDir is a path to the camera intrinsic parameters.

				//load robot 3D pose.
				std::string rootJoints = ui.rootDir + "/csv";//path to the eyeHand directory.
				std::string file_joints = "joints.csv";
				file_joints = "/" + file_joints;
				file_joints = rootJoints + file_joints;
				std::vector<std::vector<double>> pose_robot = readCSV(file_joints);//(n_data,{x,y,z,nx,ny,nz}) -> homogeneous matrix
				int num_pose = pose_robot.size();
				std::cout << "num_pose=" << num_pose << std::endl;
				std::vector<cv::Mat> Hs_tcp2base(num_pose);//H_(input_frame)2(output_frame)
				//get transform matrix.
				for (int idx = 0; idx < num_pose; idx++) {
					pose_robot[idx][0] *= 1000.0;//x [m] -> [mm]
					pose_robot[idx][1] *= 1000.0;//y [m] -> [mm]
					pose_robot[idx][2] *= 1000.0;//z [m] -> [mm]
					Hs_tcp2base[idx] = createHomogeneousMatrix(pose_robot[idx]);
				}

				//2.corner detection and show display to check whether we'll reverse the points.
				chess2camera.main(ui.rootDir); //transform the object points from the object frame(chessboard frame) to the camera frame.
				//convert translation and rotation vector to homogeneous matrix
				std::vector<cv::Mat> Hs_chess2camera(num_pose);//
				for (int idx = 0; idx < num_pose; idx++) {
					std::vector<double> pose(6);
					for (int j = 0; j < 3; j++)
						pose[j] = chess2camera.tvecs_left[idx].at<double>(j);
					for (int j = 0; j < 3; j++)
						pose[j + 3] = chess2camera.rvecs_left[idx].at<double>(j);
					Hs_chess2camera[idx] = createHomogeneousMatrix(pose);
				}

				//cv::calibrateHandEye() -> get transformation matrix.
				std::vector<cv::Mat> R_base2tcp(num_pose), t_base2tcp(num_pose), R_chess2camera(num_pose), t_chess2camera(num_pose);
				cv::Mat R_cam2base, t_cam2base, H_base2tcp;
				for (int idx = 0; idx < num_pose; idx++) {
					//chess pose in the base frame.
					cv::invert(Hs_tcp2base[idx], H_base2tcp);
					R_base2tcp[idx] = H_base2tcp(cv::Rect(0, 0, 3, 3)).clone();
					t_base2tcp[idx] = H_base2tcp(cv::Rect(3, 0, 1, 3)).clone();
					//invert H_chess2camera
					R_chess2camera[idx] = Hs_chess2camera[idx](cv::Rect(0, 0, 3, 3));
					t_chess2camera[idx] = Hs_chess2camera[idx](cv::Rect(3, 0, 1, 3));
				}

				//calibrateHandEye
				cv::calibrateHandEye(R_base2tcp, t_base2tcp, R_chess2camera, t_chess2camera, R_cam2base, t_cam2base, cv::CALIB_HAND_EYE_TSAI);
				cv::Mat H_cam2base = cv::Mat::zeros(4, 4, CV_64F);//camera pose in a robot base frame.
				H_cam2base.at<double>(3, 3) = 1.0;
				R_cam2base.copyTo(H_cam2base(cv::Rect(0, 0, 3, 3)));
				t_cam2base.copyTo(H_cam2base(cv::Rect(3, 0, 1, 3)));

				//save transformation matrix
				// Open a CSV file for writing
				std::ofstream file_transform("transform_camera2base.csv");
				if (!file_transform.is_open()) {
					std::cerr << "Error opening file for writing." << std::endl;
				}
				// Write the matrix data to the CSV file
				for (int row = 0; row < H_cam2base.rows; ++row) {
					for (int col = 0; col < H_cam2base.cols; ++col) {
						file_transform << H_cam2base.at<double>(row, col);
						if (col < H_cam2base.cols - 1) {
							file_transform << ",";  // Separate values with commas
						}
					}
					file_transform << "\n";  // New line after each row
				}
				file_transform.close();

				//Evaluation
				//triangulate points of chessboard corners
				std::vector<cv::Point3d> points_camera;//points in a camera frame.
				tri.cal3D_eyehand(chess2camera.points_left, chess2camera.points_right, 0, points_camera);//dlt
				for (int i = 0; i < points_camera.size(); i++) {
					std::cout << "point-" << i << " :: x=" << points_camera[i].x << " mm, y=" << points_camera[i].y << ", z=" << points_camera[i].z << " mm" << std::endl;
				}
				//transform from a camera frame to the robot base frame.
				cv::Mat P, P_base, P_base_inv, P_diff;
				for (int idx = 0; idx < points_camera.size(); idx++) {
					P = (cv::Mat_<double>(4, 1) <<
						points_camera[idx].x,
						points_camera[idx].y,
						points_camera[idx].z,
						1);  // Homogeneous coordinates
					P_base = H_cam2base * P;
					std::cout << "estimated points=" << P_base << std::endl;
					std::cout << "true points=" << Hs_tcp2base[idx](cv::Rect(3, 0, 1, 4)) << std::endl;
					P_diff = P_base - Hs_tcp2base[idx](cv::Rect(3, 0, 1, 4));
					double norm_diff = cv::norm(P_diff);
					std::cout << "----- error=" << norm_diff << " mm --------- " << std::endl;
				}
			}
		}
		else if (type_process == 2) {
			std::cout << "You chose Evaluation of depth accuracy. If it's okay type \'c\'. If you want to change mode, type \'q\'" << std::endl;
			std::cin >> bool_mode;
			//save data
			if (bool_mode.compare(sign_continue) == 0) {
				//evaluation of stereo vision.
				//std::string rootDir = "C:/Users/kawaw/cpp/eyeToHand_calibration/eyeToHand_calibration/camera_calibration";
				std::string rootDir = "C:/Users/kawaw/cpp/eyeToHand_calibration/eyeToHand_calibration/camera_calibration";
				std::string file_intrinsic_left = "camera0_intrinsics.dat"; 
				file_intrinsic_left = "/" + file_intrinsic_left;
				file_intrinsic_left = rootDir + file_intrinsic_left;
				std::string file_intrinsic_right = "camera1_intrinsics.dat";
				file_intrinsic_right = "/" + file_intrinsic_right;
				file_intrinsic_right = rootDir + file_intrinsic_right;
				std::string file_extrinsic_left = "camera0_rot_trans.dat";
				file_extrinsic_left = "/" + file_extrinsic_left;
				file_extrinsic_left = rootDir + file_extrinsic_left;
				std::string file_extrinsic_right = "camera1_rot_trans.dat";
				file_extrinsic_right = "/" + file_extrinsic_right;
				file_extrinsic_right = rootDir + file_extrinsic_right;
				Triangulation tri(file_intrinsic_left,file_intrinsic_right,file_extrinsic_left,file_extrinsic_right);//Triangulation
				tri.main();
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