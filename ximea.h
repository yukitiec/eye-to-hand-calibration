#pragma once

//#include "stdafx.h"
#include "xiApiPlusOcv.hpp"
#pragma comment(lib, "xiapi64.lib")

class Ximea : public xiAPIplusCameraOcv {
public:
	/**
	* @brief コンストラクタ　トリガケーブル使用
	*
	* @param[in] width 画像幅
	* @param[in] height 画像高さ
	* @param[in] framerate 撮影フレームレート
	* @param[in] serialNumber カメラのシリアル番号(0の時，接続されているカメラを探してポートが一番上に接続されたカメラを使用)
	* @param[in] expTime_us 露光時間(μs)
	* @param[in] isBinning ビニングを使うかどうか
	* @param[in] isFollower トリガケーブル接続時のマスタとスレーブの指定
	*/
	Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning, const bool& isFollower);
	/**
	* @brief コンストラクタ　露光時間指定
	*
	* @param[in] width 画像幅
	* @param[in] height 画像高さ
	* @param[in] framerate 撮影フレームレート
	* @param[in] serialNumber カメラのシリアル番号(0の時，接続されているカメラを探してポートが一番上に接続されたカメラを使用)
	* @param[in] expTime_us 露光時間(μs)
	* @param[in] isBinning ビニングを使うかどうか
	* @param[in] capMode 画像取得モード選択(0:モノクロ，1カラー)
	*/
	Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning, int capMode);
	/**
	* @brief コンストラクタ　露光時間をフレームレートから計算
	*
	* @param[in] width 画像幅
	* @param[in] height 画像高さ
	* @param[in] framerate 撮影フレームレート
	* @param[in] serialNumber カメラのシリアル番号(0の時，接続されているカメラを探してポートが一番上に接続されたカメラを使用)
	* @param[in] isBinning ビニングを使うかどうか
	*/
	Ximea(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const bool& isBinning);

	/**
	* @brief モノクロ画像取得かどうか
	*
	* @return　true：モノクロ，false：カラー
	*/
	bool isCapMono() const
	{
		return isCapMono_;
	}

private:
	const int capMode_; //! 画像取得モード(0:モノクロ，1カラー)
	bool isCapMono_;	//! 画像取得がモノクロかどうか
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
	int initCamera(const unsigned int& width, const unsigned int& height, const unsigned int& framerate, const std::string& serialNumber, const unsigned int& expTime_us, const bool& isBinning);

	/**
	* @brief 画像の幅や高さが最大以下でステップに適合しているかチェックし，あっていない時は合うように設定
	*
	* @param[in] size チェックしたい幅，もしくは高さ
	* @param[in] maxSize 最大値
	* @param[in] step 値が設定可能なステップ
	* @return 調整されたサイズ
	*/
	int limitSizeStep(const unsigned int& size, const unsigned int& maxSize, const unsigned int& step);
};