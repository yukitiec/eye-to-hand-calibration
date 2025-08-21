#include "display_info.h"

//�}�E�X�̈ʒu���擾
static void on_mouse(int event, int x, int y, int flags, void *param)
{
	cv::Point2d *pos = (cv::Point2d *) param;
	if (event == cv::EVENT_LBUTTONDOWN) {
		pos->x = x;
		pos->y = y;
	}
}

//DispInfo
/**
* @brief �R���X�g���N�^
* @param[in] queDispInfo �\���f�[�^
* @param[in] width �摜�̕�
* @param[in] height �摜�̍���
* @param[in] imgMono �摜�����m�N�����ǂ���(false�F�J���[�Ctrue�F���m�N��)
*/
DispInfo::DispInfo(decltype(queDispInfo_) &queDispInfo, const unsigned int width, const unsigned int height, bool imgMono=true)
	: queDispInfo_(queDispInfo)
	, width_(width), height_(height), clkPos_({ -1, -1 }), lastClkPos_({ -1,-1 }), isTerminate_(false)
	, threshs_({30,30}), imgMono_(imgMono)
{}

/**
* @brief �摜�C�g���b�L���O�ʒu�C�������x�\��
*
* @param[out] isSaveImage �摜��ۑ�����(s)
*
* @note �摜���\�����ꂽ�E�C���h�E���t�H�[�J�X������ԂŃ{�^�����������Ƃŉ��L�̗l�ɓ���D
* - "q": �v���O�����I��
* - "s": �摜��ۑ��J�n(�f�o�b�O�p)
* - "S": �摜�ۑ����~
*/
const int DispInfo::operator()(std::atomic<bool>& isSaveImage)
{
	const std::string winName = "image";
	cv::namedWindow(winName);
	//�}�E�X�R�[���o�b�N 
	//cv::setMouseCallback(winName, on_mouse, (void *)&clkPos_); //�}�X�N�摜�őI��

	dispData_t dispData;
	std::array < cv::Mat3b, 2> rgbSrcs;
	cv::Mat3b disp;
	unsigned long frameCount = 0, prevFrameCount = 0;
	cv::Scalar markColor[5] = { { 51, 0, 230 },{ 225,169, 44 },{ 118, 202,147 },{ 0, 200, 252 },{ 124, 13, 154 } }; //�g���b�L���O�_�\���F
	const cv::Point imgCenter(width_ / 2.0, height_ / 2.0);
	const double dispFontSize = 0.8; //�\�������T�C�Y

	using timeT = std::chrono::high_resolution_clock;
	auto startT = timeT::now();

//	cv::VideoWriter writer("video.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(2*width_, height_));
	cv::Mat3b outputImage;

	while (cv::getWindowProperty(winName, 0) >= 0) {
		//�f�[�^�ǂݍ���
		if (!queDispInfo_.pop(dispData)) {
			std::this_thread::yield();
			continue;
		}

		const auto elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(timeT::now() - startT).count() / 1000;
		startT = timeT::now();
		//�\��
		frameCount = dispData.frameCount;
		cv::cvtColor(dispData.srcs[0], rgbSrcs[0], cv::COLOR_GRAY2BGR);
		cv::cvtColor(dispData.srcs[1], rgbSrcs[1], cv::COLOR_GRAY2BGR);
		
		//������rgbSrc�ɕ`�擙


		//�\��
		auto fps = (frameCount - prevFrameCount) * 1000.0 / elapsed_ms;
		cv::setWindowTitle(winName, std::to_string(fps) + " fps");

		cv::hconcat(rgbSrcs[0], rgbSrcs[1], disp);
		//cv::hconcat(dispData.srcs[0], dispData.srcs[1], disp);
		cv::imshow(winName, disp);
		auto key = cv::waitKey(1);
		if (key == 'q') {
			break;
		}
		else if (key == 's') {
			isSaveImage = true;
		}
		else if (key == 'S') {
			isSaveImage = false;
		}
		prevFrameCount = frameCount;
	}
	terminateDisplay();
	return 0;
}

/**
* @brief �ŐV�̃N���b�N�ʒu���擾
*
* @param [out] pos �ŐV�̃N���b�N�ʒu
*
* @retval 0 �擾����
* @retval 1 �O�񂩂�X�V�Ȃ�
*/
int DispInfo::getClkPos(cv::Point2d & pos)
{
	pos = clkPos_;
	if (clkPos_ == lastClkPos_) {
		return 1;
	}
	lastClkPos_ = clkPos_;
	return 0;
}

/**
	* @brief �X���C�_�Őݒ肵����l��臒l�̎擾
	*
	* @param [in] camIndex �J�����ԍ�
	*
	* @return 臒l
	*/
int DispInfo::getBinThresh(int camIndex) const
{
	return threshs_[camIndex];
}
