#include "stdafx.h"


//#include "ximea.h"
/**
* @brief �R���X�g���N�^�@�g���K�P�[�u���g�p
*
* @param[in] width �摜��
* @param[in] height �摜����
* @param[in] framerate �B�e�t���[�����[�g
* @param[in] serialNumber �J�����̃V���A���ԍ�(0�̎��C�ڑ�����Ă���J������T���ă|�[�g����ԏ�ɐڑ����ꂽ�J�������g�p)
* @param[in] expTime_us �I������ (��s)
* @param[in] isBinning �r�j���O���g�����ǂ���
* @param[in] isFollower �g���K�P�[�u���ڑ����̃}�X�^�ƃX���[�u�̎w��
*/
Ximea::Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning = true, const bool& isFollwer = false)
	:capMode_(0)
{

	initCamera(width, height, framerate, serialNumber, expTime_us, isBinning);

	// �g���K
	if (!isFollwer) {
		// ���[�_�[
		SetTriggerSource(XI_TRG_OFF);
		SetGPOSelector(XI_GPO_PORT1);
		SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);
	}
	else {
		// �t�H�����[
		SetTriggerSource(XI_TRG_EDGE_RISING);
		SetGPISelector(XI_GPI_PORT1);
		SetGPIMode(XI_GPI_TRIGGER);
	}

	//�N��
	std::cout << "Starting acquisition..." << std::endl;
	StartAcquisition();
	std::cout << "Camera started!!" << std::endl;
}
/**
* @brief �R���X�g���N�^�@�I�����Ԏw��
*
* @param[in] width �摜��
* @param[in] height �摜����
* @param[in] framerate �B�e�t���[�����[�g
* @param[in] serialNumber �J�����̃V���A���ԍ�(0�̎��C�ڑ�����Ă���J������T���ă|�[�g����ԏ�ɐڑ����ꂽ�J�������g�p)
* @param[in] expTime_ms �I������ (��s)
* @param[in] isBinning �r�j���O���g�����ǂ���
* @param[in] capMode �摜�擾���[�h�I��(0:���m�N���C1�J���[)
*/
Ximea::Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning = true, int capMode = 0)
	:capMode_(capMode), isCapMono_(true)
{
	initCamera(width, height, framerate, serialNumber, expTime_us, isBinning);

	//�N��
	std::cout << "Starting acquisition..." << std::endl;
	StartAcquisition();
	std::cout << "Camera started!!" << std::endl;
}
/**
* @brief �R���X�g���N�^�@�I�����Ԃ��t���[�����[�g����v�Z
*
* @param[in] width �摜��
* @param[in] height �摜����
* @param[in] framerate �B�e�t���[�����[�g
* @param[in] serialNumber �J�����̃V���A���ԍ�(0�̎��C�ڑ�����Ă���J������T���ă|�[�g����ԏ�ɐڑ����ꂽ�J�������g�p)
* @param[in] isBinning �r�j���O���g�����ǂ���
*/
Ximea::Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const bool& isBinning = true)
	:capMode_(0)
{
	const int expOffset = 210; //�I�����Ԃ̃I�t�Z�b�g�@framerate����̑S�I�����ԈȊO�ɓ��������ő������Ԃ������邽�߁C���@�̓�������������K�v�D
	const unsigned int expTime_us = static_cast<int>(1000.0 * 1000.0 / framerate) - expOffset;

	initCamera(width, height, framerate, serialNumber, expTime_us, isBinning);

	//�N��
	std::cout << "Starting acquisition..." << std::endl;
	StartAcquisition();
	std::cout << "Camera started!!" << std::endl;
}

/**
* @brief �J�����̏������i�J�����ڑ��C�摜�T�C�Y�C�r�j���O�C�t���[�����[�g�C�I�����Ԃ̐ݒ�j
*
* @param[in] width �摜��
* @param[in] height �摜����
* @param[in] framerate �B�e�t���[�����[�g
* @param[in] serialNumber �J�����̃V���A���ԍ�
* @param[in] expTime_us �I������(��s)
* @param[in] isBinning �r�j���O���邩�ǂ���
*/
int Ximea::initCamera(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning)
{
	std::cout << "Initialize camera" << std::endl;
	//�J�����ڑ�
	char serialChar[24];
	std::cout << serialNumber << std::endl;
	if (serialNumber == "0") {
		OpenFirst();
	}
	else {
		OpenBySN(serialNumber.c_str());
	}
	GetSerialNumber(serialChar, sizeof(serialChar) / sizeof(serialChar[0]));
	std::cout << "Opening camera[" << +serialChar << "]..." << std::endl;

	if (IsSensorColor() == 1 && capMode_ == 1) {
		SetImageDataFormat(XI_IMG_FORMAT::XI_RGB24);
		isCapMono_ = false;
		std::cout << "Capture color image" << std::endl;
	}
	else {
		SetImageDataFormat(XI_IMG_FORMAT::XI_MONO8);
		std::cout << "Capture grayscale image" << std::endl;
	}

	SetSensorFeatureSelector(XI_SENSOR_FEATURE_ZEROROT_ENABLE);
	SetSensorFeatureValue(1000);

	// �r�j���O
	//�ő�T�C�Y�擾
	const auto maxWidth = GetWidth_Maximum();
	const auto maxHeight = GetHeight_Maximum();
	if (isBinning && width <= maxWidth * 0.5 && height <= maxHeight * 0.5) {
		SetDownsampling(XI_DWN_2x2);
		SetDownsamplingType(XI_SKIPPING);
	}

	// �摜�T�C�Y
	const auto widthStep = GetWidth_Increment();
	const auto heightStep = GetHeight_Increment();
	SetWidth(limitSizeStep(width, maxWidth, widthStep));
	SetHeight(limitSizeStep(height, maxHeight, heightStep));
	std::cout << "Image Width: " << GetWidth() << std::endl;
	std::cout << "Image Height: " << GetHeight() << std::endl;

	// �I�t�Z�b�g�@���S�g�p
	SetOffsetX(GetOffsetX_Maximum() / 2);
	SetOffsetY(GetOffsetY_Maximum() / 2);
	std::cout << "Offset X: " << GetOffsetX() << std::endl;
	std::cout << "Offset Y: " << GetOffsetY() << std::endl;

	// �t���[�����[�g�C�I������
	SetAcquisitionTimingMode(XI_ACQ_TIMING_MODE_FRAME_RATE);
	//�t���[�����[�g�͈̔̓`�F�b�N
	//�I�����ԂɈˑ����āC�t���[�����[�g�̍ő�C�ŏ����ς��̂łƂ肠�����������I�����ԓ����
	SetExposureTime(100);
	auto frameRate = static_cast<float>(framerate);
	// �ő�C�ŏ��ɂ��Ă��Ă��܂��Ɛ����Ɉ������������ہC�I�������������Ȃ�C�_�ł��Ă��܂��̂ŁC
	// 10%�ʃ}�[�W��������D
	const auto maxFR = static_cast<float>(GetFrameRate_Maximum() * 0.9);
	const auto minFR = static_cast<float>(GetFrameRate_Minimum() * 1.1);
	frameRate = std::min(frameRate, maxFR);
	frameRate = std::max(frameRate, minFR);
	SetFrameRate(frameRate);
	const unsigned int maxExpTime = 1000.0 * 1000.0 / frameRate * 0.9; //���ۂ̃t���[������ő�I�����Ԍv�Z(�摜�]�������l������90%�ɐݒ�)
	//std::cout << maxExpTime << std::endl;
	SetExposureTime(std::min(expTime_us, maxExpTime));
	SetFrameRate(frameRate);
	//std::cout << "Framerate: " << minFR << " - " << maxFR << std::endl;
	std::cout << "Frame rate: " << GetFrameRate() << std::endl;
	std::cout << "Exp. Time: " << GetExposureTime() << std::endl;

	//�I�����ԂƃQ�C�����Œ�
	DisableAutoExposureAutoGain();

	return 0;
}
/**
* @brief �摜�̕��⍂�����ő�ȉ��ŃX�e�b�v�ɓK�����Ă��邩�`�F�b�N���C�����Ă��Ȃ����͍����悤�ɐݒ�
*
* @param[in] size �`�F�b�N���������C�������͍���
* @param[in] maxSize �ő�l
* @param[in] step �l���ݒ�\�ȃX�e�b�v
* @return �������ꂽ�T�C�Y
*/
int Ximea::limitSizeStep(const unsigned int& size, const unsigned int& maxSize, const unsigned int& step)
{
	auto checkedSize = std::min(size, maxSize); //�ő�l����
	//�X�e�b�v�̊m�F
	if (checkedSize % step != 0) {
		checkedSize -= step - size % step;
	}
	return checkedSize;
}
