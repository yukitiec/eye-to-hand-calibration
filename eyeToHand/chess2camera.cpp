#include "chess2camera.h"

void Chess2camera::main() {

    //1,get chessboard information
    std::cout << ":: Chessboard information :: \n width=";
    std::cin >> width;
    std::cout << "height=";
    std::cin >> height;
    std::cout << "size=";
    std::cin >> size;
    std::cout << std::endl;
}

void Chess2camera::getCorners(const cv::Mat& frame, std::vector<cv::Point2f>& corners,
    const bool automatic
    ){

	// Convert to grayscale
	cv::Mat gray;
	cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

	// Find chessboard corners
	std::vector<cv::Point2f> corners_tmp,corners_tmp2;
	bool found = cv::findChessboardCorners(gray, cv::Size(width, height), corners,
		cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
	if (found) {
		// Refine corner locations
		cv::cornerSubPix(gray, corners, cv::Size(N_NEIGHBOR, N_NEIGHBOR), cv::Size(-1, -1),
			cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.00001));
		
        // Draw circles on detected corners
        int counter_corner = 0; int bool_reverse = 0;//not reverse.
        cv::Mat image_draw = frame.clone();
        for (const auto& corner : corners) {
            if (counter_corner == 0)//first corner
                cv::circle(image_draw, corner, 5, cv::Scalar(255, 0, 0), -1);//B
            else//others
                cv::circle(image_draw, corner, 5, cv::Scalar(0, 0, std::abs((counter_corner * 20) % 255)), -1);//R
            counter_corner++;
        }

        // Show the image with detected corners
        //cv::imwrite("detected.png", image_draw);
        cv::imshow(winName, image_draw);
        cv::waitKey(10);
        if (!automatic) {
            std::cout << "Will you reverse corners order? 0:Okay, 1:reverse corners, 2:change direction, 3:reverse and change direction :: " << std::endl;
            std::cin >> bool_reverse;
        }
        else
            bool_reverse = 2;//2->0

        // Reverse the vector
        corners_tmp = corners;
        int counter_reverse = 0;
        while (true) {
            if (bool_reverse == 0) break;
            corners_tmp = corners;
            //reverse
            if (bool_reverse == 1) {
                std::reverse(corners_tmp.begin(), corners_tmp.end());
                image_draw = frame.clone();
                counter_corner = 0;
                for (const auto& corner : corners_tmp) {
                    if (counter_corner == 0)//first corner
                        cv::circle(image_draw, corner, 5, cv::Scalar(255, 0, 0), -1);//B
                    else//others
                        cv::circle(image_draw, corner, 5, cv::Scalar(0, 0, std::abs((255 - counter_corner * 50) % 255)), -1);//R
                    counter_corner++;
                }
            }
            else if (bool_reverse == 2) {//other direction
                int idx = 0; int max_idx;
                for (int row = 0; row < height; row++) {//for each row
                    max_idx = (row + 1) * width - 1;//max index
                    for (int col = 0; col < width; col++) {//for each column
                        idx = row * width + col;
                        corners_tmp[idx] = corners[max_idx - col];
                    }
                }
                //save
                image_draw = frame.clone();
                counter_corner = 0;
                for (const auto& corner : corners_tmp) {
                    if (counter_corner == 0)//first corner
                        cv::circle(image_draw, corner, 5, cv::Scalar(255, 0, 0), -1);//B
                    else//others
                        cv::circle(image_draw, corner, 5, cv::Scalar(0, 0, std::abs((counter_corner * 50) % 255)), -1);//R
                    counter_corner++;
                }
            }
            else if (bool_reverse == 3) {//other direction and reverse
                corners_tmp2 = corners;
                std::reverse(corners_tmp2.begin(), corners_tmp2.end());
                int idx = 0; int max_idx;
                for (int row = 0; row < height; row++) {//for each row
                    max_idx = row * width - 1;//max index
                    for (int col = 0; col < width; col++) {//for each column
                        idx = row * width + col;
                        corners_tmp[idx] = corners_tmp2[max_idx - col];
                    }
                }
                //save
                image_draw = frame.clone();
                counter_corner = 0;
                for (const auto& corner : corners_tmp) {
                    if (counter_corner == 0)//first corner
                        cv::circle(image_draw, corner, 5, cv::Scalar(255, 0, 0), -1);//B
                    else//others
                        cv::circle(image_draw, corner, 5, cv::Scalar(0, 0, std::abs((255 - counter_corner * 50) % 255)), -1);//R
                    counter_corner++;
                }
            }
            //cv::imwrite("detected.png", image_draw);
            if (!automatic) {
                std::cout << "Is this Okay? 0:Okay, 1:reverse corners, 2:change direction, 3:reverse and change direction :: " << std::endl;
                std::cin >> bool_reverse;
            }
            else {
                if (counter_reverse == 0)
                    bool_reverse = 1;
                else if (counter_reverse==1)
                    bool_reverse = 0;//2->0
                counter_reverse += 1;
            }

            cv::imshow(winName, image_draw);
            cv::waitKey(3);
        }
        //close windows
        //cv::destroyWindow("Detected Corners");

        //save original points
        corners = corners_tmp;
	}
}