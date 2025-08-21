/**
* @brief 型定義
*/
#pragma once
using streoImage = std::array<cv::Mat1b, 2>; //!ステレオ画像
using saveImages_t = std::vector<streoImage>; //!画像保存用配列
constexpr unsigned int maxSaveImageNum = 2000; //!最大画像保存数

//! @brief 表示データの型
struct dispData_t {
	unsigned long frameCount;
	std::array<cv::Mat1b, 2> srcs;
};

constexpr std::size_t capacity = 3; //! spsc_queueのサイズ
template <typename T>
using spsc_queue = boost::lockfree::spsc_queue<T, boost::lockfree::capacity<capacity>>; //! スレッド間通信用のロックフリーキューの型

//送受信関係
using sendData_t = std::array<double, 10>; //! 送信データの型
using recvData_t = std::array<double, 5>; //! 受信データの型