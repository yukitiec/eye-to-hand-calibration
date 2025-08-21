/**
* @brief �^��`
*/
#pragma once
using streoImage = std::array<cv::Mat1b, 2>; //!�X�e���I�摜
using saveImages_t = std::vector<streoImage>; //!�摜�ۑ��p�z��
constexpr unsigned int maxSaveImageNum = 2000; //!�ő�摜�ۑ���

//! @brief �\���f�[�^�̌^
struct dispData_t {
	unsigned long frameCount;
	std::array<cv::Mat1b, 2> srcs;
};

constexpr std::size_t capacity = 3; //! spsc_queue�̃T�C�Y
template <typename T>
using spsc_queue = boost::lockfree::spsc_queue<T, boost::lockfree::capacity<capacity>>; //! �X���b�h�ԒʐM�p�̃��b�N�t���[�L���[�̌^

//����M�֌W
using sendData_t = std::array<double, 10>; //! ���M�f�[�^�̌^
using recvData_t = std::array<double, 5>; //! ��M�f�[�^�̌^