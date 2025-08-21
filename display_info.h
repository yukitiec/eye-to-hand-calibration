/**
* @brief�@�摜�C�d�S�ʒu�C�������x�̕\��
*/

#pragma once

/**
* @struct DispInfo
* @brief ���\���N���X
*/
struct DispInfo {
private:
	spsc_queue<dispData_t> &queDispInfo_; //! �\���f�[�^�̃��b�N�t���[�L���[
	const unsigned int height_; //! �\���摜�̍���
	const unsigned int width_; //! �\���摜�̕�
	cv::Point2d clkPos_; //!�}�E�X���N���b�N�����ʒu
	cv::Point2d lastClkPos_; //! �O��擾�����N���b�N�ʒu(getClkPos�ōX�V)
	bool isTerminate_; //!�I��
	std::array<int, 2> threshs_; //! 臒l�i�X���C�_�Őݒ�j
	const bool imgMono_;	//! �摜�J���[(true:���m�N���Cfalse�F�J���[) 

	/**
	* @brief �����I��
	*
	* @retval 0 �I��
	*/
	int terminateDisplay() {
		isTerminate_ = true;
		return 0;
	}

public:
	/**
	* @brief �R���X�g���N�^
	* @param[in] queDispInfo �\���f�[�^ 
	* @param[in] width �摜�̕�
	* @param[in] height �摜�̍���
	* @param[in] imgMono �摜�����m�N�����ǂ���(false�F�J���[�Ctrue�F���m�N��)
	*/
	DispInfo(decltype(queDispInfo_) &queDispInfo, const unsigned int width, const unsigned int height, bool imgColor);

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
	const int operator()(std::atomic<bool> &isSaveImage);

	/**
	* @brief �ŐV�̃N���b�N�ʒu���擾
	* 
	* @param [out] pos �ŐV�̃N���b�N�ʒu
	*
	* @retval 0 �擾����
	* @retval 1 �O�񂩂�X�V�Ȃ� 
	*/
	int getClkPos(cv::Point2d &pos);

	/**
	* @brief �����I���m�F
	*
	* @retval true �����I��
	* @retval false �����I�����ĂȂ�
	*/
	bool isTerminate() {
		return isTerminate_;
	}

	/**
	* @brief �X���C�_�Őݒ肵����l��臒l�̎擾
	*
	* @param [in] camIndex �J�����ԍ�
	*
	* @return 臒l
	*/
	int getBinThresh(int camIndex) const;
};