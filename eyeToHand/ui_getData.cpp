#include "ui_getData.h"

std::vector<std::vector<double>> UI_getData::loadPosesCSV(const std::string& path) {
    std::vector<std::vector<double>> poses;
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open CSV file: " << path << std::endl;
        return poses;
    }

    std::string line;
    bool firstLine = true;

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string item;
        std::vector<double> pose;

        while (std::getline(ss, item, ',')) {
            try {
                pose.push_back(std::stod(item));
            }
            catch (...) {
                pose.clear();
                break;
            }
        }

        // first row is a header, skip it
        if (firstLine) {
            firstLine = false;
            if (pose.size() != 6) {
                continue;
            }
        }

        if (pose.size() == 6) {
            //convert degree to radian.
            std::vector<double> pose_rad;
            for (double j_deg : pose)
            {
                double j_rad = j_deg * PI / 180.0;
                pose_rad.push_back(j_rad);
            }
            poses.push_back(pose_rad);
        }
        else {
            std::cerr << "Skipped invalid CSV row: " << line << std::endl;
        }
    }

    std::cout << "Loaded " << poses.size() << " target joint poses from CSV." << std::endl;
    return poses;
}

void UI_getData::main(std::string joints_replay_path) {

    // make directory for saving data of eye to hand calibration
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

    // directory strings
    rootDir = "eyeHand";
    imgDir_left = "eyeHand/img/left";
    imgDir_right = "eyeHand/img/right";
    undistortDir_left = "eyeHand/img/undistort/left";
    undistortDir_right = "eyeHand/img/undistort/right";
    csvDir = "eyeHand/csv";

    int counter;
    std::string bool_skip;
    std::string sign_skip = "s";
    std::string file_path, name_file, file_left, file_right, file_csv;

    std::vector<double> joints_ur, tcp_ur;
    std::array<cv::Mat1b, 2> frames;
    int frameIndex;

    std::cout << "If you wanna gather data, type 'c'. if you wanna skip, type 's': ";
    std::cin >> bool_skip;
    if (bool_skip.compare(sign_skip) == 0) {
        return;
    }

    // input start counter
    std::cout << "input start counter :: ";
    std::cin >> counter;
    std::cout << std::endl;

    // input joint CSV path
    std::string poseCsvPath = joints_replay_path;

    // load target poses
    std::vector<std::vector<double>> targetPoses = loadPosesCSV(poseCsvPath);
    if (targetPoses.empty()) {
        std::cerr << "No valid target poses loaded. Abort." << std::endl;
        return;
    }

    // make a one holistic csv file
    name_file = "joints.csv";
    file_path = csvDir + "/" + name_file;
    std::ofstream file_whole(file_path);
    if (!file_whole.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return;
    }

    std::cout << "start getting data" << std::endl;

    std::string winName_left = "current image (left)";
    cv::namedWindow(winName_left);
    std::string winName_right = "current image (right)";
    cv::namedWindow(winName_right);

    for (size_t i = 0; i < targetPoses.size(); ++i) {
        const std::vector<double>& targetJoints = targetPoses[i];

        std::cout << "========================================" << std::endl;
        std::cout << "Moving to target pose " << (i + 1) << " / " << targetPoses.size() << std::endl;
        std::cout << "Target joints = ";
        for (double q : targetJoints) {
            std::cout << q << ", ";
        }
        std::cout << std::endl;

        // move robot directly to target joints
        urCtrl->moveJ(targetJoints, 0.5, 0.5);
        urCtrl->stopJ();

        std::cout << "wait a few seconds ........" << std::endl;
        cv::waitKey(3000);

        // show current images
        if (!queueFrame.empty() && !queueFrameIndex.empty()) {
            ut.getImagesYolo(frames, frameIndex);
            cv::imshow(winName_left, frames[0]);
            cv::imshow(winName_right, frames[1]);
            cv::waitKey(100);
        }

        // get latest images again before saving
        if (!queueFrame.empty() && !queueFrameIndex.empty()) {
            ut.getImagesYolo(frames, frameIndex);
        }
        else {
            std::cerr << "Frame queue is empty. Skipping image save for index " << counter << std::endl;
            continue;
        }

        // save left and right frames
        std::ostringstream filenameStream_left, filenameStream_right, filenameStream_csv;

        filenameStream_left << std::setw(2) << std::setfill('0') << counter << ".png";
        file_left = imgDir_left + "/" + filenameStream_left.str();
        cv::imwrite(file_left, frames[0]);

        filenameStream_right << std::setw(2) << std::setfill('0') << counter << ".png";
        file_right = imgDir_right + "/" + filenameStream_right.str();
        cv::imwrite(file_right, frames[1]);

        // save TCP pose
        tcp_ur = urDI->getActualTCPPose();

        filenameStream_csv << std::setw(2) << std::setfill('0') << counter << ".csv";
        file_csv = csvDir + "/" + filenameStream_csv.str();

        std::ofstream file(file_csv);
        if (!file.is_open()) {
            std::cerr << "Error opening per-sample csv: " << file_csv << std::endl;
        }
        else {
            for (int j = 0; j < tcp_ur.size(); j++) {
                file << tcp_ur[j];
                file_whole << tcp_ur[j];
                if (j < static_cast<int>(tcp_ur.size()) - 1) {
                    file << ",";
                    file_whole << ",";
                }
            }
            file << "\n";
            file_whole << "\n";
            file.close();
        }

        std::cout << "########## SAVE " << counter << "-th data ##########" << std::endl;
        counter++;
    }

    file_whole.close();
    cv::destroyWindow(winName_left);
    cv::destroyWindow(winName_right);

    std::cout << "finish getting data" << std::endl;
}

void UI_getData::makeDir(std::filesystem::path& dirPath) {
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