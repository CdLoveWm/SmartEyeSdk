#pragma once
#include <stereocamera.h>
#include "Common.h"
#include "framemonitor.h"
#include "CameraFrameMonitor.h"

/// <summary>
/// 双目相机Handler
/// 单例模式
/// </summary>
class SmartEyeHandler
{
private:
	static SmartEyeHandler* _instance;
	SmartEyeHandler() {};
	static StereoCamera* _camera;
	static CameraFrameMonitor* frameMonitor;
	void SetObscaleParam(int imgType);
public:
	static SmartEyeHandler* GetInstance();
	string GetRootPath();
	string GetCurrentDateStr(bool hasMillsec = false);
	bool IsCameraConnected();
	bool InitCamera(const char* ipAddr);
	bool ReleaseCamera();
	FrameResult* GetFrame(int rows, int cols, int imgType, bool isYolo, unsigned char* ret_data);
	bool CollectFrame(int rows, int cols, int imgType);
	cv::Mat GetFrame(int imgType, FrameResult* res);
};

