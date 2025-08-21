#include "triangulation.h"


void Triangulation::main() {
    /**
    *@brief main loop for getting images and calculate 3D position.
    *
    */
    //make directory for saving data of eye to hand calibration
    std::filesystem::path path_root, path_img, path_img_left, path_img_right, path_undistort_left, path_undistort_right, path_csv;
    std::string rootDir, imgDir_left, imgDir_right, undistortDir_left, undistortDir_right, csvDir;
    path_root = "depth";
    path_img = "depth/img";
    path_img_left = "depth/img/left";
    path_img_right = "depth/img/right";
    path_undistort_left = "depth/img/left/undistort";
    path_undistort_right = "depth/img/right/undistort";
    path_csv = "depth/csv";
    makeDir(path_root);
    makeDir(path_img);
    makeDir(path_img_left);
    makeDir(path_img_right);
    makeDir(path_undistort_left);
    makeDir(path_undistort_right);
    makeDir(path_csv);

    //directory
    rootDir = "depth";
    imgDir_left = "depth/img/left";
    imgDir_right = "depth/img/right";
    undistortDir_left = "depth/img/undistort/left";
    undistortDir_right = "depth/img/undistort/right";
    csvDir = "depth/csv";

    int counter;
    std::string bool_save, bool_continue;
    std::string file_path, name_file;
    std::vector<double> joints_ur, tcp_ur;
    std::array<cv::Mat1b, 2> frames;
    int frameIndex;
    double xCenter, yCenter;
    cv::Point2d p_left, p_right;
    std::vector<cv::Point2d> ps_left, ps_right;
    std::vector<cv::Point3d> results;

    //make a one holistic csv file.
    name_file = "position_3d.csv";
    name_file = "/" + name_file;
    file_path = csvDir + name_file;
    std::ofstream file_whole(file_path);
    if (!file_whole.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
    }

    if (q_startTracking.empty())
    {
        while (true)
        {
            //std::cout << "q_startTracking.empty()" << q_startTracking.empty() << std::endl;
            if (!q_startTracking.empty())
            {
                break;
            }
        }
    }

    std::cout << "start  getting data" << std::endl;
    std::cout << "input start counter :: ";
    std::cin >> counter;
    std::cout << std::endl;

    while (true) {

        //hear whether save image and TCP pose.
        std::cout << "Will you calculate 3D position of the target?" << std::endl;
        std::cout << "type \'s\' if you wanna save, and otherwise type \'n\':";
        std::cin >> bool_save;

        //save data
        if (bool_save.compare(sign_save) == 0) {
            //get images
            if (!queueFrame.empty() && !queueFrameIndex.empty()) {
                ut.getImagesYolo(frames, frameIndex);
            }
            //initialize
            ps_left.clear(); ps_right.clear();
            
            //left
            // Display the image
            // Select ROI
            cv::Rect roi_left = cv::selectROI("Select ROI of the left image", frames[0]);
            xCenter = roi_left.x + roi_left.width / 2.0;
            yCenter = roi_left.y + roi_left.height / 2.0;
            p_left.x = xCenter; p_left.y = yCenter;
            ps_left.push_back(p_left);
            cv::destroyWindow("Select ROI of the left image");

            //right

            // Select ROI
            cv::Rect roi_right = cv::selectROI("Select ROI of the right image", frames[1]);
            xCenter = roi_right.x + roi_right.width / 2.0;
            yCenter = roi_right.y + roi_right.height / 2.0;
            p_right.x = xCenter; p_right.y = yCenter;
            ps_right.push_back(p_right);
            cv::destroyWindow("Select ROI of the right image");

            //calculate 3D position.
            cal3D(ps_left, ps_right, 0, results);//DLT
            cal3D(ps_left, ps_right, 1, results);//stereo
            std::cout << "DLT :: x=" << results[0].x << "mm , y=" << results[0].y << "mm , z=" << results[0].z<<" mm" << std::endl;
            std::cout << "Stereo :: x=" << results[1].x << "mm , y=" << results[1].y << "mm , z=" << results[1].z << " mm" << std::endl;

            //save frame_left and frame_wright.
            name_file = std::to_string(counter) + ".png";
            name_file = "/" + name_file;
            file_path = imgDir_left + "/" + name_file;
            cv::imwrite(file_path, frames[0]);
            file_path = imgDir_right + "/" + name_file;
            cv::imwrite(file_path, frames[1]);

            for (int j = 0; j < results.size(); j++) {
                file_whole << results[j].x<<","<< results[j].y<<","<< results[j].z;
                if (j < results.size() - 1) {
                    file_whole << ",";
                }
            }
            file_whole << "\n";

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
    std::cout << "finish getting data" << std::endl;
}

void Triangulation::cal3D(std::vector<cv::Point2d>& pts_left, std::vector<cv::Point2d>& pts_right, int method_triangulate,std::vector<cv::Point3d>& results)
{
   
    //std::cout << "3" << std::endl;
    if (method_triangulate == 0) {//DLT (Direct Linear Translation)
        dlt(pts_left, pts_right, results);
    }
    else if (method_triangulate == 1) {//stereo triangulation
        stereo3D(pts_left, pts_right, results);
    }
}

void Triangulation::dlt(std::vector<cv::Point2d>& points_left, std::vector<cv::Point2d>& points_right, std::vector<cv::Point3d>& results)
{
    /**
    * @brief calculate 3D points with DLT method
    * @param[in] points_left, points_right {n_data,(xCenter,yCenter)}
    * @param[out] reuslts 3D points storage. shape is like (n_data, (x,y,z))
    */
    cv::Mat points_left_mat(points_left);
    cv::Mat undistorted_points_left_mat;
    cv::Mat points_right_mat(points_right);
    cv::Mat undistorted_points_right_mat;

    // Undistort the points
    cv::undistortPoints(points_left_mat, undistorted_points_left_mat, cameraMatrix_left, distCoeffs_left);
    cv::undistortPoints(points_right_mat, undistorted_points_right_mat, cameraMatrix_right, distCoeffs_right);

    // Reproject normalized coordinates to pixel coordinates
    cv::Mat normalized_points_left(undistorted_points_left_mat.rows, 1, CV_64FC2);
    cv::Mat normalized_points_right(undistorted_points_right_mat.rows, 1, CV_64FC2);

    for (int i = 0; i < undistorted_points_left_mat.rows; ++i) {
        double x, y;
        x = undistorted_points_left_mat.at<cv::Vec2d>(i, 0)[0];
        y = undistorted_points_left_mat.at<cv::Vec2d>(i, 0)[1];
        normalized_points_left.at<cv::Vec2d>(i, 0)[0] = cameraMatrix_left.at<double>(0, 0) * x + cameraMatrix_left.at<double>(0, 2);
        normalized_points_left.at<cv::Vec2d>(i, 0)[1] = cameraMatrix_left.at<double>(1, 1) * y + cameraMatrix_left.at<double>(1, 2);

        x = undistorted_points_right_mat.at<cv::Vec2d>(i, 0)[0];
        y = undistorted_points_right_mat.at<cv::Vec2d>(i, 0)[1];
        normalized_points_right.at<cv::Vec2d>(i, 0)[0] = cameraMatrix_right.at<double>(0, 0) * x + cameraMatrix_right.at<double>(0, 2);
        normalized_points_right.at<cv::Vec2d>(i, 0)[1] = cameraMatrix_right.at<double>(1, 1) * y + cameraMatrix_right.at<double>(1, 2);
    }

    // Output matrix for the 3D points
    cv::Mat triangulated_points_mat;

    // Triangulate points
    cv::triangulatePoints(projectMatrix_left, projectMatrix_right, normalized_points_left, normalized_points_right, triangulated_points_mat);
    //cv::triangulatePoints(projectMatrix_left, projectMatrix_right, undistorted_points_left_mat, undistorted_points_right_mat, triangulated_points_mat);

    // Convert homogeneous coordinates to 3D points
    triangulated_points_mat=triangulated_points_mat.t();
    cv::convertPointsFromHomogeneous(triangulated_points_mat.reshape(4), triangulated_points_mat);

    // Access triangulated 3D points
    results.clear();

    for (int i = 0; i < triangulated_points_mat.rows; i++) {
        cv::Point3d point;
        point.x = triangulated_points_mat.at<double>(i, 0);
        point.y = triangulated_points_mat.at<double>(i, 1);
        point.z = triangulated_points_mat.at<double>(i, 2);
        results.push_back(point);
    }
    
    // Convert from camera coordinate to robot base coordinate
    for (auto& point : results) {
        double x = point.x;
        double y = point.y;
        double z = point.z;
        point.x = transform_cam2base.at<double>(0, 0) * x + transform_cam2base.at<double>(0, 1) * y + transform_cam2base.at<double>(0, 2) * z + transform_cam2base.at<double>(0, 3);
        point.y = transform_cam2base.at<double>(1, 0) * x + transform_cam2base.at<double>(1, 1) * y + transform_cam2base.at<double>(1, 2) * z + transform_cam2base.at<double>(1, 3);
        point.z = transform_cam2base.at<double>(2, 0) * x + transform_cam2base.at<double>(2, 1)* y + transform_cam2base.at<double>(2, 2)* z + transform_cam2base.at<double>(2, 3);
    }
}

void Triangulation::stereo3D(std::vector<cv::Point2d>& left, std::vector<cv::Point2d>& right, std::vector<cv::Point3d>& results)
{
    /**
    * @brief calculate 3D points with stereo method
    * @param[in] points_left, points_right {n_data,(xCenter,yCenter)}
    * @param[out] reuslts 3D points storage. shape is like (n_data, (x,y,z))
    */

    //undistort points
    cv::Mat points_left_mat(left);
    cv::Mat undistorted_points_left_mat;
    cv::Mat points_right_mat(right);
    cv::Mat undistorted_points_right_mat;

    // Undistort the points
    cv::undistortPoints(points_left_mat, undistorted_points_left_mat, cameraMatrix_left, distCoeffs_left);
    cv::undistortPoints(points_right_mat, undistorted_points_right_mat, cameraMatrix_right, distCoeffs_right);
    std::cout << "undistorted_points_left_mat=" << undistorted_points_left_mat << std::endl;

    // Reproject normalized coordinates to pixel coordinates
    //left
    cv::Mat normalized_points_left(undistorted_points_left_mat.rows, 2, CV_64F);
    double x, y;
    for (int i = 0; i < undistorted_points_left_mat.rows; ++i) {
        x = undistorted_points_left_mat.at<cv::Vec2d>(i, 0)[0];
        y = undistorted_points_left_mat.at<cv::Vec2d>(i, 0)[1];
        normalized_points_left.at<cv::Vec2d>(i, 0)[0] = cameraMatrix_left.at<double>(0, 0) * x + cameraMatrix_left.at<double>(0, 2);
        normalized_points_left.at<cv::Vec2d>(i, 0)[1] = cameraMatrix_left.at<double>(1, 1) * y + cameraMatrix_left.at<double>(1, 2);
    }
    //right
    cv::Mat normalized_points_right(undistorted_points_right_mat.rows, 2, CV_64F);
    for (int i = 0; i < undistorted_points_right_mat.rows; ++i) {
        x = undistorted_points_right_mat.at<cv::Vec2d>(i, 0)[0];
        y = undistorted_points_right_mat.at<cv::Vec2d>(i, 0)[1];
        normalized_points_right.at<cv::Vec2d>(i, 0)[0] = cameraMatrix_right.at<double>(0, 0) * x + cameraMatrix_right.at<double>(0, 2);
        normalized_points_right.at<cv::Vec2d>(i, 0)[1] = cameraMatrix_right.at<double>(1, 1) * y + cameraMatrix_right.at<double>(1, 2);
    }

    std::cout << "normalized_points_left_mat=" << normalized_points_left << std::endl;
    int size_left = normalized_points_left.rows;
    int size_right = normalized_points_right.rows;
    int size;
    if (size_left <= size_right) size = size_left;
    else size = size_right;

    cv::Point3d result;
    for (int i = 0; i < size; i++) {
        double xl = normalized_points_left.at<double>(i, 0); double xr = normalized_points_right.at<double>(i, 0);
        double yl = normalized_points_left.at<double>(i, 1); double yr = normalized_points_left.at<double>(i, 1);
        double disparity = xl - xr;
        double X = (double)(BASELINE / disparity) * (xl - oX_left - (fSkew / fY) * (yl - oY_left));
        double Y = (double)(BASELINE * (fX / fY) * (yl - oY_left) / disparity);
        double Z = (double)(fX * BASELINE / disparity);
        /* convert Camera coordinate to robot base coordinate */
        X = transform_cam2base.at<double>(0, 0) * X + transform_cam2base.at<double>(0, 1) * Y + transform_cam2base.at<double>(0, 2) * Z + transform_cam2base.at<double>(0, 3);
        Y = transform_cam2base.at<double>(1, 0) * X + transform_cam2base.at<double>(1, 1) * Y + transform_cam2base.at<double>(1, 2) * Z + transform_cam2base.at<double>(1, 3);
        Z = transform_cam2base.at<double>(2, 0) * X + transform_cam2base.at<double>(2, 1) * Y + transform_cam2base.at<double>(2, 2) * Z + transform_cam2base.at<double>(2, 3);
        result.x = X; result.y = Y; result.z = Z;
        results.push_back(result);
    }
}

void Triangulation::cal3D_eyehand(std::vector<cv::Point2d>& pts_left, std::vector<cv::Point2d>& pts_right, int method_triangulate, std::vector<cv::Point3d>& results)
{

    //std::cout << "3" << std::endl;
    if (method_triangulate == 0) {//DLT (Direct Linear Translation)
        dlt_eyehand(pts_left, pts_right, results);
    }
    else if (method_triangulate == 1) {//stereo triangulation
        stereo3D(pts_left, pts_right, results);
    }
}

void Triangulation::dlt_eyehand(std::vector<cv::Point2d>& points_left, std::vector<cv::Point2d>& points_right, std::vector<cv::Point3d>& results)
{
    /**
    * @brief calculate 3D points with DLT method
    * @param[in] points_left, points_right {n_data,(xCenter,yCenter)}. These points are from undistorted images.
    * @param[out] reuslts 3D points storage. shape is like (n_data, (x,y,z))
    */
    cv::Mat points_left_mat(points_left);
    cv::Mat undistorted_points_left_mat;
    cv::Mat points_right_mat(points_right);
    cv::Mat undistorted_points_right_mat;

    cv::Mat points_left_homogeneous(2, points_left.size(), CV_64F);
    cv::Mat points_right_homogeneous(2, points_right.size(), CV_64F);

    for (size_t i = 0; i < points_left.size(); i++) {
        points_left_homogeneous.at<double>(0, i) = points_left[i].x;
        points_left_homogeneous.at<double>(1, i) = points_left[i].y;
        //points_left_homogeneous.at<double>(2, i) = 1.0;

        points_right_homogeneous.at<double>(0, i) = points_right[i].x;
        points_right_homogeneous.at<double>(1, i) = points_right[i].y;
        //points_right_homogeneous.at<double>(2, i) = 1.0;
    }

    // Undistort the points
    //cv::undistortPoints(points_left_mat, undistorted_points_left_mat, cameraMatrix_left, distCoeffs_left);
    //cv::undistortPoints(points_right_mat, undistorted_points_right_mat, cameraMatrix_right, distCoeffs_right);

    // Reproject normalized coordinates to pixel coordinates
    //cv::Mat normalized_points_left(undistorted_points_left_mat.rows, 1, CV_64FC2);
    //cv::Mat normalized_points_right(undistorted_points_right_mat.rows, 1, CV_64FC2);

    /*for (int i = 0; i < undistorted_points_left_mat.rows; ++i) {
        double x, y;
        x = undistorted_points_left_mat.at<cv::Vec2d>(i, 0)[0];
        y = undistorted_points_left_mat.at<cv::Vec2d>(i, 0)[1];
        normalized_points_left.at<cv::Vec2d>(i, 0)[0] = cameraMatrix_left.at<double>(0, 0) * x + cameraMatrix_left.at<double>(0, 2);
        normalized_points_left.at<cv::Vec2d>(i, 0)[1] = cameraMatrix_left.at<double>(1, 1) * y + cameraMatrix_left.at<double>(1, 2);

        x = undistorted_points_right_mat.at<cv::Vec2d>(i, 0)[0];
        y = undistorted_points_right_mat.at<cv::Vec2d>(i, 0)[1];
        normalized_points_right.at<cv::Vec2d>(i, 0)[0] = cameraMatrix_right.at<double>(0, 0) * x + cameraMatrix_right.at<double>(0, 2);
        normalized_points_right.at<cv::Vec2d>(i, 0)[1] = cameraMatrix_right.at<double>(1, 1) * y + cameraMatrix_right.at<double>(1, 2);
    }*/

    // Output matrix for the 3D points
    cv::Mat triangulated_points_mat;

    // Triangulate points
    //cv::triangulatePoints(projectMatrix_left, projectMatrix_right, normalized_points_left, normalized_points_right, triangulated_points_mat);
    cv::triangulatePoints(projectMatrix_left, projectMatrix_right, points_left_homogeneous, points_right_homogeneous, triangulated_points_mat);

    // Convert homogeneous coordinates to 3D points
    triangulated_points_mat = triangulated_points_mat.t();

    cv::Mat points_3d;
    cv::convertPointsFromHomogeneous(triangulated_points_mat.reshape(4), points_3d);//input should be 4 channels inputs.-> triangulated_points_mat : (4,20)-> .reshape(4) : (1,20,4). channels are 4.

    // Access triangulated 3D points
    results.clear();

    cv::Point3d point;
    for (int i = 0; i < points_3d.rows; i++) {
        point = points_3d.at<cv::Point3d>(i);
        results.push_back(point);
    }

    // Convert from camera coordinate to robot base coordinate
    for (auto& point : results) {
        double x = point.x;
        double y = point.y;
        double z = point.z;
        point.x = transform_cam2base.at<double>(0, 0) * x + transform_cam2base.at<double>(0, 1) * y + transform_cam2base.at<double>(0, 2) * z + transform_cam2base.at<double>(0, 3);
        point.y = transform_cam2base.at<double>(1, 0) * x + transform_cam2base.at<double>(1, 1) * y + transform_cam2base.at<double>(1, 2) * z + transform_cam2base.at<double>(1, 3);
        point.z = transform_cam2base.at<double>(2, 0) * x + transform_cam2base.at<double>(2, 1) * y + transform_cam2base.at<double>(2, 2) * z + transform_cam2base.at<double>(2, 3);
    }
}

void Triangulation::makeDir(std::filesystem::path& dirPath) {
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