#pragma once

//#include "stdafx.h"
#include "xiApiPlusOcv.hpp"
#pragma comment(lib, "xiapi64.lib")

class Ximea : public xiAPIplusCameraOcv {
public:
	/**
	* @brief �R���X�g���N�^�@�g���K�P�[�u���g�p
	*
	* @param[in] width �摜��
	* @param[in] height �摜����
	* @param[in] framerate �B�e�t���[�����[�g
	* @param[in] serialNumber �J�����̃V���A���ԍ�(0�̎��C�ڑ�����Ă���J������T���ă|�[�g����ԏ�ɐڑ����ꂽ�J�������g�p)
	* @param[in] expTime_us �I������(��s)
	* @param[in] isBinning �r�j���O���g�����ǂ���
	* @param[in] isFollower �g���K�P�[�u���ڑ����̃}�X�^�ƃX���[�u�̎w��
	*/
	Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning, const bool& isFollower);
	/**
	* @brief �R���X�g���N�^�@�I�����Ԏw��
	*
	* @param[in] width �摜��
	* @param[in] height �摜����
	* @param[in] framerate �B�e�t���[�����[�g
	* @param[in] serialNumber �J�����̃V���A���ԍ�(0�̎��C�ڑ�����Ă���J������T���ă|�[�g����ԏ�ɐڑ����ꂽ�J�������g�p)
	* @param[in] expTime_us �I������(��s)
	* @param[in] isBinning �r�j���O���g�����ǂ���
	* @param[in] capMode �摜�擾���[�h�I��(0:���m�N���C1�J���[)
	*/
	Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning, int capMode);
	/**
	* @brief �R���X�g���N�^�@�I�����Ԃ��t���[�����[�g����v�Z
	*
	* @param[in] width �摜��
	* @param[in] height �摜����
	* @param[in] framerate �B�e�t���[�����[�g
	* @param[in] serialNumber �J�����̃V���A���ԍ�(0�̎��C�ڑ�����Ă���J������T���ă|�[�g����ԏ�ɐڑ����ꂽ�J�������g�p)
	* @param[in] isBinning �r�j���O���g�����ǂ���
	*/
	Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const bool& isBinning);

	/**
	* @brief ���m�N���摜�擾���ǂ���
	*
	* @return�@true�F���m�N���Cfalse�F�J���[
	*/
	bool isCapMono() const
	{
		return isCapMono_;
	}

private:
	const int capMode_; //! �摜�擾���[�h(0:���m�N���C1�J���[)
	bool isCapMono_;	//! �摜�擾�����m�N�����ǂ���
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
	int initCamera(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning);

	/**
	* @brief �摜�̕��⍂�����ő�ȉ��ŃX�e�b�v�ɓK�����Ă��邩�`�F�b�N���C�����Ă��Ȃ����͍����悤�ɐݒ�
	*
	* @param[in] size �`�F�b�N���������C�������͍���
	* @param[in] maxSize �ő�l
	* @param[in] step �l���ݒ�\�ȃX�e�b�v
	* @return �������ꂽ�T�C�Y
	*/
	int limitSizeStep(const unsigned int& size, const unsigned int& maxSize, const unsigned int& step);
};