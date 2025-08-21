#include "stdafx.h"


//#include "ximea.h"
/**
* @brief コンストラクタ　トリガケーブル使用
*
* @param[in] width 画像幅
* @param[in] height 画像高さ
* @param[in] framerate 撮影フレームレート
* @param[in] serialNumber カメラのシリアル番号(0の時，接続されているカメラを探してポートが一番上に接続されたカメラを使用)
* @param[in] expTime_us 露光時間 (μs)
* @param[in] isBinning ビニングを使うかどうか
* @param[in] isFollower トリガケーブル接続時のマスタとスレーブの指定
*/
Ximea::Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning = true, const bool& isFollwer = false)
	:capMode_(0)
{

	initCamera(width, height, framerate, serialNumber, expTime_us, isBinning);

	// トリガ
	if (!isFollwer) {
		// リーダー
		SetTriggerSource(XI_TRG_OFF);
		SetGPOSelector(XI_GPO_PORT1);
		SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);
	}
	else {
		// フォロワー
		SetTriggerSource(XI_TRG_EDGE_RISING);
		SetGPISelector(XI_GPI_PORT1);
		SetGPIMode(XI_GPI_TRIGGER);
	}

	//起動
	std::cout << "Starting acquisition..." << std::endl;
	StartAcquisition();
	std::cout << "Camera started!!" << std::endl;
}
/**
* @brief コンストラクタ　露光時間指定
*
* @param[in] width 画像幅
* @param[in] height 画像高さ
* @param[in] framerate 撮影フレームレート
* @param[in] serialNumber カメラのシリアル番号(0の時，接続されているカメラを探してポートが一番上に接続されたカメラを使用)
* @param[in] expTime_ms 露光時間 (μs)
* @param[in] isBinning ビニングを使うかどうか
* @param[in] capMode 画像取得モード選択(0:モノクロ，1カラー)
*/
Ximea::Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning = true, int capMode = 0)
	:capMode_(capMode), isCapMono_(true)
{
	initCamera(width, height, framerate, serialNumber, expTime_us, isBinning);

	//起動
	std::cout << "Starting acquisition..." << std::endl;
	StartAcquisition();
	std::cout << "Camera started!!" << std::endl;
}
/**
* @brief コンストラクタ　露光時間をフレームレートから計算
*
* @param[in] width 画像幅
* @param[in] height 画像高さ
* @param[in] framerate 撮影フレームレート
* @param[in] serialNumber カメラのシリアル番号(0の時，接続されているカメラを探してポートが一番上に接続されたカメラを使用)
* @param[in] isBinning ビニングを使うかどうか
*/
Ximea::Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const bool& isBinning = true)
	:capMode_(0)
{
	const int expOffset = 210; //露光時間のオフセット　framerateからの全露光時間以外に内部処理で多少時間がかかるため，実機の動作を見つつ調整が必要．
	const unsigned int expTime_us = static_cast<int>(1000.0 * 1000.0 / framerate) - expOffset;

	initCamera(width, height, framerate, serialNumber, expTime_us, isBinning);

	//起動
	std::cout << "Starting acquisition..." << std::endl;
	StartAcquisition();
	std::cout << "Camera started!!" << std::endl;
}

/**
* @brief カメラの初期化（カメラ接続，画像サイズ，ビニング，フレームレート，露光時間の設定）
*
* @param[in] width 画像幅
* @param[in] height 画像高さ
* @param[in] framerate 撮影フレームレート
* @param[in] serialNumber カメラのシリアル番号
* @param[in] expTime_us 露光時間(μs)
* @param[in] isBinning ビニングするかどうか
*/
int Ximea::initCamera(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning)
{
	std::cout << "Initialize camera" << std::endl;
	//カメラ接続
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

	// ビニング
	//最大サイズ取得
	const auto maxWidth = GetWidth_Maximum();
	const auto maxHeight = GetHeight_Maximum();
	if (isBinning && width <= maxWidth * 0.5 && height <= maxHeight * 0.5) {
		SetDownsampling(XI_DWN_2x2);
		SetDownsamplingType(XI_SKIPPING);
	}

	// 画像サイズ
	const auto widthStep = GetWidth_Increment();
	const auto heightStep = GetHeight_Increment();
	SetWidth(limitSizeStep(width, maxWidth, widthStep));
	SetHeight(limitSizeStep(height, maxHeight, heightStep));
	std::cout << "Image Width: " << GetWidth() << std::endl;
	std::cout << "Image Height: " << GetHeight() << std::endl;

	// オフセット　中心使用
	SetOffsetX(GetOffsetX_Maximum() / 2);
	SetOffsetY(GetOffsetY_Maximum() / 2);
	std::cout << "Offset X: " << GetOffsetX() << std::endl;
	std::cout << "Offset Y: " << GetOffsetY() << std::endl;

	// フレームレート，露光時間
	SetAcquisitionTimingMode(XI_ACQ_TIMING_MODE_FRAME_RATE);
	//フレームレートの範囲チェック
	//露光時間に依存して，フレームレートの最大，最小が変わるのでとりあえず小さい露光時間入れる
	SetExposureTime(100);
	auto frameRate = static_cast<float>(framerate);
	// 最大，最小にしてしてしまうと制限に引っかかった際，露光がおかしくなり，点滅してしまうので，
	// 10%位マージンを入れる．
	const auto maxFR = static_cast<float>(GetFrameRate_Maximum() * 0.9);
	const auto minFR = static_cast<float>(GetFrameRate_Minimum() * 1.1);
	frameRate = std::min(frameRate, maxFR);
	frameRate = std::max(frameRate, minFR);
	SetFrameRate(frameRate);
	const unsigned int maxExpTime = 1000.0 * 1000.0 / frameRate * 0.9; //実際のフレームから最大露光時間計算(画像転送等も考慮して90%に設定)
	//std::cout << maxExpTime << std::endl;
	SetExposureTime(std::min(expTime_us, maxExpTime));
	SetFrameRate(frameRate);
	//std::cout << "Framerate: " << minFR << " - " << maxFR << std::endl;
	std::cout << "Frame rate: " << GetFrameRate() << std::endl;
	std::cout << "Exp. Time: " << GetExposureTime() << std::endl;

	//露光時間とゲインを固定
	DisableAutoExposureAutoGain();

	return 0;
}
/**
* @brief 画像の幅や高さが最大以下でステップに適合しているかチェックし，あっていない時は合うように設定
*
* @param[in] size チェックしたい幅，もしくは高さ
* @param[in] maxSize 最大値
* @param[in] step 値が設定可能なステップ
* @return 調整されたサイズ
*/
int Ximea::limitSizeStep(const unsigned int& size, const unsigned int& maxSize, const unsigned int& step)
{
	auto checkedSize = std::min(size, maxSize); //最大値制限
	//ステップの確認
	if (checkedSize % step != 0) {
		checkedSize -= step - size % step;
	}
	return checkedSize;
}
