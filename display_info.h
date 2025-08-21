/**
* @brief　画像，重心位置，処理速度の表示
*/

#pragma once

/**
* @struct DispInfo
* @brief 情報表示クラス
*/
struct DispInfo {
private:
	spsc_queue<dispData_t> &queDispInfo_; //! 表示データのロックフリーキュー
	const unsigned int height_; //! 表示画像の高さ
	const unsigned int width_; //! 表示画像の幅
	cv::Point2d clkPos_; //!マウス左クリックした位置
	cv::Point2d lastClkPos_; //! 前回取得したクリック位置(getClkPosで更新)
	bool isTerminate_; //!終了
	std::array<int, 2> threshs_; //! 閾値（スライダで設定）
	const bool imgMono_;	//! 画像カラー(true:モノクロ，false：カラー) 

	/**
	* @brief 処理終了
	*
	* @retval 0 終了
	*/
	int terminateDisplay() {
		isTerminate_ = true;
		return 0;
	}

public:
	/**
	* @brief コンストラクタ
	* @param[in] queDispInfo 表示データ 
	* @param[in] width 画像の幅
	* @param[in] height 画像の高さ
	* @param[in] imgMono 画像がモノクロかどうか(false：カラー，true：モノクロ)
	*/
	DispInfo(decltype(queDispInfo_) &queDispInfo, const unsigned int width, const unsigned int height, bool imgColor);

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
	const int operator()(std::atomic<bool> &isSaveImage);

	/**
	* @brief 最新のクリック位置を取得
	* 
	* @param [out] pos 最新のクリック位置
	*
	* @retval 0 取得成功
	* @retval 1 前回から更新なし 
	*/
	int getClkPos(cv::Point2d &pos);

	/**
	* @brief 処理終了確認
	*
	* @retval true 処理終了
	* @retval false 処理終了してない
	*/
	bool isTerminate() {
		return isTerminate_;
	}

	/**
	* @brief スライダで設定した二値化閾値の取得
	*
	* @param [in] camIndex カメラ番号
	*
	* @return 閾値
	*/
	int getBinThresh(int camIndex) const;
};