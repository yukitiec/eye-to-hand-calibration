// ximea_test.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "global_parameters.h"
#include "calibration.h"
#include "realsense.h"

#pragma warning(suppress : 4996)


int main()
{
	
	//constructor
	Utility ut;
	Calibration _calibration;

	//RealSense image 
	RealSense _rs_instance;
	std::thread th_rs(&RealSense::main, &_rs_instance);

	//calibration.
	_calibration.main();


	return 0;
}

