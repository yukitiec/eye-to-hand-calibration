// ximea_test.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include "global_parameters.h"
#include "calibration.h"

#pragma warning(suppress : 4996)


/* frame queue */
extern std::queue<std::array<cv::Mat1b, 2>> queueFrame;
extern std::queue<int> queueFrameIndex;

extern std::queue<bool> q_startTracking; //start tracking
extern std::queue<bool> q_endTracking; //end tracking

//#include "communicate_dspace.h"
//#include "display_info.h"


/**
* @brief �T�C�Y���l������ROI���쐬
*
* @param[in] center �g���b�L���O�_�̑O�t���[���̈ʒu�iROI�̒��S�j
* @param[in] src ���͉摜
* @param[in] roiBase ROI�̘g
* @return ROI�摜
*/
cv::Mat createROI(cv::Point2d& center, const cv::Mat& src, const cv::Rect& roiBase) {
	return src((roiBase + cv::Point(center)) & cv::Rect(cv::Point(0, 0), src.size()));
}

int main()
{
	
	//constructor
	Utility ut;

	std::string file_camera_params = "C:/Users/kawaw/cpp/eyeToHand_calibration/eyeToHand_calibration/camera_calibration/downsample_512";
	Calibration calibration(file_camera_params);


	//�J�����̃p�����[�^�ݒ�
	const unsigned int imgWidth = 512; //�摜�̕�
	const unsigned int imgHeight = 512;//�摜�̍���
	const unsigned int frameRate = 10; //�t���[�����[�g
	const unsigned int expTime_us = 3000;// 792; //�I������
	const int imgGain = 20;
	const bool isBinning = true;
	const int capMode = 0; //0:���m�N���C1:�J���[
	const std::string leaderCamID = "30958851";		//���[�_�[�J����
	const std::string followerCamID = "30957651"; 	//�t�H�����J����
	std::atomic<bool> isSaveImage = false;//save img
	const int InitialFrame = 4800; //initial frame for saving

	//std::thread thread_skeleton(&Skeleton::main,skeleton,std::ref(q_startTracking),std::ref(q_endTracking));
	std::thread thread_cal(&Calibration::main, calibration);
#if SYNC_CAMERAS
	std::array<Ximea, 2> cams = { Ximea(imgWidth, imgHeight, frameRate, leaderCamID, expTime_us, isBinning, false), Ximea(imgWidth, imgHeight, frameRate, followerCamID, expTime_us, isBinning, true) };
#else
	std::array<Ximea, 2> cams = { Ximea(imgWidth, imgHeight, frameRate, leaderCamID, expTime, isBinning, 0), Ximea(imgWidth, imgHeight, frameRate, followerCamID, expTime, isBinning,0) };
#endif //SYNC_CAMERAS
	//�Q�C���ݒ�
	cams[0].SetGain(imgGain);
	cams[1].SetGain(imgGain);


	int saveCount = 0;
	std::array<cv::Mat1b, 2> srcs = { cv::Mat1b::zeros(imgHeight, imgWidth),cv::Mat1b::zeros(imgHeight, imgWidth) }; //�擾�摜


	//���C������
	for (int fCount = 0;; fCount++) {
		for (int i_i = 0; i_i < cams.size(); i_i++) {
			srcs[i_i] = cams[i_i].GetNextImageOcvMat(); //�摜�擾
		}

		if (saveCount >= maxSaveImageNum)
		{
			std::cout << "finish" << std::endl;
			break;
		}

		ut.pushImg(srcs, fCount);//push images to que
		if (fCount == 0)
			q_startTracking.push(true);

	}
	if (!q_startTracking.empty()) q_startTracking.pop();
	//�摜�o��
	if (!q_endTracking.empty()) q_endTracking.pop();

	std::cout << "Finish main proc" << std::endl;
	//display
	//dispThread.join();
	//thread_skeleton.join();
	thread_cal.join();
	std::cout << "Skeleton Tracking has finished" << std::endl;
	//sendThread.join();
	//recvThread.join();
	return 0;
}

