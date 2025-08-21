#include "ui_getData.h"

void UI_getData::main() {

    //make directory for saving data of eye to hand calibration
    path_root = "eyeHand";
    path_img = "eyeHand/img";
    path_img_left = "eyeHand/img/left";
    path_img_right = "eyeHand/img/right";
    path_undistort_left = "eyeHand/img/left/undistort";
    path_undistort_right = "eyeHand/img/right/undistort";
    path_csv = "eyeHand/csv";
    makeDir(path_root);
    makeDir(path_img);
    makeDir(path_img_left);
    makeDir(path_img_right);
    makeDir(path_undistort_left);
    makeDir(path_undistort_right);
    makeDir(path_csv);
    //directory
    rootDir = "eyeHand";
    imgDir_left = "eyeHand/img/left";
    imgDir_right = "eyeHand/img/right";
    undistortDir_left = "eyeHand/img/undistort/left";
    undistortDir_right = "eyeHand/img/undistort/right";
    csvDir = "eyeHand/csv";

    int counter;
    std::string bool_save, bool_continue;
    std::string file_path, name_file,file_left,file_right,file_csv;
    std::vector<double> joints_ur, tcp_ur;
    std::array<cv::Mat1b, 2> frames;
    int frameIndex;

    std::string bool_skip; std::string sign_skip = "s";
    std::cout << "If you wanna gather data, type \'c\'. if you wanna skip,  type \'s\':";
    std::cin >> bool_skip;
    if (bool_skip.compare(sign_skip) != 0) {

        //make a one holistic csv file.
        name_file = "joints.csv";
        name_file = "/" + name_file;
        file_path = csvDir + name_file;
        std::ofstream file_whole(file_path);
        if (!file_whole.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
        }

        std::cout << "start  getting data" << std::endl;
        std::cout << "input start counter :: ";
        std::cin >> counter;
        std::cout << std::endl;

        std::string winName_left = "current image (left)";
        cv::namedWindow(winName_left);
        std::string winName_right = "current image (right)";
        cv::namedWindow(winName_right);

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
            //move robot with moveJ()
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
            std::cout << "wait a few seconds ........" << std::endl;
            if (!queueFrame.empty() && !queueFrameIndex.empty()) {
                ut.getImagesYolo(frames, frameIndex);
                cv::imshow(winName_left, frames[0]);
                cv::imshow(winName_right, frames[1]);
                cv::waitKey(2000);
            }

            //hear whether save image and TCP pose.
            std::cout << "Will you save images and TCP pose?" << std::endl;
            std::cout << "type \'s\' if you wanna save, and otherwise type \'n\':";
            std::cin >> bool_save;

            //save data
            if (bool_save.compare(sign_save) == 0) {
                //get images
                if (!queueFrame.empty() && !queueFrameIndex.empty()) {
                    ut.getImagesYolo(frames, frameIndex);
                }

                //save frame_left and frame_wright.
                std::ostringstream filenameStream_left, filenameStream_right, filenameStream_csv;
                filenameStream_left << std::setw(2) << std::setfill('0') << counter << ".png";
                file_left = filenameStream_left.str();
                file_left = "/" + file_left;
                file_left = imgDir_left + file_left;
                cv::imwrite(file_left, frames[0]);
                filenameStream_right << std::setw(2) << std::setfill('0') << counter << ".png";
                file_right = filenameStream_right.str();
                file_right = "/" + file_right;
                file_right = imgDir_right + file_right;
                cv::imwrite(file_right, frames[1]);

                //save joints angle
                tcp_ur = urDI->getActualTCPPose();
                filenameStream_csv << std::setw(2) << std::setfill('0') << counter << ".csv";
                file_csv = filenameStream_csv.str();
                file_csv = "/" + file_csv;
                file_csv = csvDir + file_csv;
                std::ofstream file(file_csv);
                for (int j = 0; j < tcp_ur.size(); j++) {
                    file << tcp_ur[j];
                    file_whole << tcp_ur[j];
                    if (j < tcp_ur.size() - 1) {
                        file << ",";
                        file_whole << ",";
                    }
                }
                file << "\n";
                file_whole << "\n";
                file.close();

                std::cout << "########## SAVE " << counter << "-th data ########## " << std::endl;
                counter++;
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
        file_whole.close();
        cv::destroyWindow(winName_left);
        cv::destroyWindow(winName_right);
        std::cout << "finish getting data" << std::endl;
    }
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