#include "display_info.h"

//マウスの位置を取得
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
* @brief コンストラクタ
* @param[in] queDispInfo 表示データ
* @param[in] width 画像の幅
* @param[in] height 画像の高さ
* @param[in] imgMono 画像がモノクロかどうか(false：カラー，true：モノクロ)
*/
DispInfo::DispInfo(decltype(queDispInfo_) &queDispInfo, const unsigned int width, const unsigned int height, bool imgMono=true)
	: queDispInfo_(queDispInfo)
	, width_(width), height_(height), clkPos_({ -1, -1 }), lastClkPos_({ -1,-1 }), isTerminate_(false)
	, threshs_({30,30}), imgMono_(imgMono)
{}

/**
* @brief 画像，トラッキング位置，処理速度表示
*
* @param[out] isSaveImage 画像を保存する(s)
*
* @note 画像が表示されたウインドウをフォーカスした状態でボタンを押すことで下記の様に動作．
* - "q": プログラム終了
* - "s": 画像を保存開始(デバッグ用)
* - "S": 画像保存を停止
*/
const int DispInfo::operator()(std::atomic<bool>& isSaveImage)
{
	const std::string winName = "image";
	cv::namedWindow(winName);
	//マウスコールバック 
	//cv::setMouseCallback(winName, on_mouse, (void *)&clkPos_); //マスク画像で選択

	dispData_t dispData;
	std::array < cv::Mat3b, 2> rgbSrcs;
	cv::Mat3b disp;
	unsigned long frameCount = 0, prevFrameCount = 0;
	cv::Scalar markColor[5] = { { 51, 0, 230 },{ 225,169, 44 },{ 118, 202,147 },{ 0, 200, 252 },{ 124, 13, 154 } }; //トラッキング点表示色
	const cv::Point imgCenter(width_ / 2.0, height_ / 2.0);
	const double dispFontSize = 0.8; //表示文字サイズ

	using timeT = std::chrono::high_resolution_clock;
	auto startT = timeT::now();

//	cv::VideoWriter writer("video.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(2*width_, height_));
	cv::Mat3b outputImage;

	while (cv::getWindowProperty(winName, 0) >= 0) {
		//データ読み込み
		if (!queDispInfo_.pop(dispData)) {
			std::this_thread::yield();
			continue;
		}

		const auto elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(timeT::now() - startT).count() / 1000;
		startT = timeT::now();
		//表示
		frameCount = dispData.frameCount;
		cv::cvtColor(dispData.srcs[0], rgbSrcs[0], cv::COLOR_GRAY2BGR);
		cv::cvtColor(dispData.srcs[1], rgbSrcs[1], cv::COLOR_GRAY2BGR);
		
		//ここでrgbSrcに描画等


		//表示
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
* @brief 最新のクリック位置を取得
*
* @param [out] pos 最新のクリック位置
*
* @retval 0 取得成功
* @retval 1 前回から更新なし
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
	* @brief スライダで設定した二値化閾値の取得
	*
	* @param [in] camIndex カメラ番号
	*
	* @return 閾値
	*/
int DispInfo::getBinThresh(int camIndex) const
{
	return threshs_[camIndex];
}
