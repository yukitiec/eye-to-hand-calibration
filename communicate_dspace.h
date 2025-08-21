/**
* @brief　dSpaceとの通信(送受信)
*/

#pragma once

#include "stdafx.h"

/**
* @struct comDspace
* @brief　dSpaceとの通信クラス
*/
struct ComDspace{

private:
	using Udp = boost::asio::ip::udp;
	spsc_queue<sendData_t> &queSend_; //! 送信データのロックフリーキュー
	spsc_queue<recvData_t> &queRecv_; //! 受信データのロックフリーキュー
	const std::string destIP_; //! 送信先(dSpace)IPアドレス
	const std::string destPort_; //! 送信先のポート
	const std::string srcPort_; //! 送信元のポート（受信用）

	bool isFinished_; //! 処理を終了するフラグ

public:
	/**
	* @brief コンストラクタ
	* @param[in] queSend 送信データ
	* @param[out] queRecv 受信データ
	* @param[in] destIP 送信先(dSpace)IPアドレス
	* @param[in] destPort 送信先(dSpace)ポート
	* @param[in] srcPort 受信(PC)ポート
	*/
	ComDspace(spsc_queue<sendData_t> &queSend, spsc_queue<recvData_t> &queRecv, const std::string &destIP, const std::string &destPort, const std::string &srcPort);

	/**
	* @brief dspaceへデータ送信
	*/
	void sendData();

	/**
	* @brief dspaceからデータ受信
	*/
	void recieveData();

	/**
	* @brief　処理を終了する
	*/
	int finishProc() {
		isFinished_ = true;
		return 0;
	}

	/**
	* @brief　終了フラグの確認
	*/
	bool isFinishProc() {
		return isFinished_;
	}
};