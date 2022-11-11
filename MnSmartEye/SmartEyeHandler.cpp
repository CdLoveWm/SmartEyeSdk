#pragma warning(disable:4996)
#include <iostream>
#include <mutex>
#include "SmartEyeHandler.h"
#include "taskiddef.h"
#include "frameid.h"
#include "CameraFrameMonitor.h"
#include "roadwaypainter.h"
#include "ObstaclePainter.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//#include "Distinguish.h"
#include "framemonitor.h"
#include <direct.h>
#include "SystemParam.h"
#include <chrono>
#include <ctime>
#include <satpext.h>
#include <iostream>
#include <fstream>
#include "Common.h"


#ifdef __POSIX__
#include <unistd.h>
#include <dirent.h>
#else
#include <io.h>
#include <direct.h>
#endif // __POSIX__

using namespace std::chrono;

std::mutex singleton_mutex;// 互斥量
SmartEyeHandler* SmartEyeHandler::_instance = NULL;
StereoCamera* SmartEyeHandler::_camera = NULL;
CameraFrameMonitor* SmartEyeHandler::frameMonitor = NULL;

/// <summary>
/// 获取Handler实例
/// </summary>
/// <returns></returns>
SmartEyeHandler* SmartEyeHandler::GetInstance()
{
	if (_instance == nullptr) {
		std::unique_lock<std::mutex> lockMutex(singleton_mutex);
		if (_instance == nullptr) {
			_instance = new SmartEyeHandler();
		}
	}
	return _instance;
}
/// <summary>
/// 相机是否连接
/// </summary>
/// <returns></returns>
bool SmartEyeHandler::IsCameraConnected()
{
	return _camera != nullptr && _camera->isConnected();
}
/// <summary>
/// 初始化相机
/// </summary>
/// <param name="ipAddr"></param>
/// <returns></returns>
bool SmartEyeHandler::InitCamera(const char* ipAddr)
{
	if (IsCameraConnected())
		return true;
	_camera = StereoCamera::connect(ipAddr);
	frameMonitor = new CameraFrameMonitor();
	return IsCameraConnected();
}
/// <summary>
/// 释放相机资源，断开连接
/// </summary>
/// <returns></returns>
bool SmartEyeHandler::ReleaseCamera()
{
	if (IsCameraConnected())
	{
		_camera->disconnectFromServer();
	}
	frameMonitor->Releae();
	return true;

	delete _camera;
	delete frameMonitor;
}
/// <summary>
/// 设置障碍物显示参数
/// </summary>
void SmartEyeHandler::SetObscaleParam(int imgType) {
	switch (imgType)
	{
	case ImageType::CalibLeft_Obstacle_OnlyDistance:
	case ImageType::CalibLeft_LaneExt_Obstacle_OnlyDistance:
		frameMonitor->showDetials = false;
		frameMonitor->singleObs = false;
		break;
	case ImageType::CalibLeft_Obstacle_Details:
	case ImageType::CalibLeft_LaneExt_Obstacle_Details:
		frameMonitor->showDetials = true;
		frameMonitor->singleObs = false;
		break;
	case ImageType::CalibLeft_Obstacle_OnlyDistance_Nearest:
	case ImageType::CalibLeft_LaneExt_Obstacle_OnlyDistance_Nearest:
		frameMonitor->showDetials = false;
		frameMonitor->singleObs = true;
		break;
	case ImageType::CalibLeft_Obstacle_Details_Nearest:
	case ImageType::CalibLeft_LaneExt_Obstacle_Details_Nearest:
		frameMonitor->showDetials = true;
		frameMonitor->singleObs = true;
		break;
	default:
		break;
	}
}
/// <summary>
/// 获取图像数据
/// </summary>
/// <param name="imgType"></param>
/// <param name="res"></param>
/// <returns></returns>
cv::Mat SmartEyeHandler::GetFrame(int imgType, FrameResult* res)
{
	cv::Mat frame;

	if (!IsCameraConnected())
	{
		res->success = false;
		res->message = "camera disconnect";
		return frame;
	}

	frameMonitor->waitForFrames(); // 等待Frame

	// 根据不同的参数类型，请求不同的数据
	switch (imgType)
	{
	case ImageType::CalibLeftCamera: // 左相机校准图
	{
		_camera->enableTasks(TaskId::DisplayTask);
		_camera->requestFrame(frameMonitor, FrameId::CalibLeftCamera);
		frame = frameMonitor->getFrameMat(FrameId::CalibLeftCamera);
	}
	break;
	case ImageType::RightCamera:
	{
		_camera->enableTasks(TaskId::DisplayTask);
		_camera->requestFrame(frameMonitor, FrameId::RightCamera);
		frame = frameMonitor->getFrameMat(FrameId::RightCamera);
	}
	break;
	case ImageType::Disparity:
	{
		_camera->enableTasks(TaskId::DisplayTask);
		_camera->requestFrame(frameMonitor, FrameId::Disparity);
		frame = frameMonitor->getFrameMat(FrameId::Disparity);
	}
	break;
	case ImageType::CalibLeft_Obstacle_OnlyDistance:
	case ImageType::CalibLeft_Obstacle_Details:
	case ImageType::CalibLeft_Obstacle_OnlyDistance_Nearest:
	case ImageType::CalibLeft_Obstacle_Details_Nearest:
	{
		SetObscaleParam(imgType);
		_camera->enableTasks(TaskId::DisplayTask | TaskId::ObstacleTask);
		_camera->requestFrame(frameMonitor, FrameId::Compound);
		frame = frameMonitor->getFrameMat(FrameId::Compound);

	}
	break;
	// 左+车道线
	case ImageType::CalibLeft_LaneExt:
	{
		_camera->enableTasks(TaskId::DisplayTask | TaskId::LaneTask);
		_camera->requestFrame(frameMonitor, FrameId::CalibLeftCamera | FrameId::LaneExt);
		frame = frameMonitor->getFrameMat(FrameId::CalibLeftCamera);
		LdwDataPack ldwDataPack = frameMonitor->getFrameLane(FrameId::LaneExt);

		if (ldwDataPack.softStatus >= 0) {
			bool roadwayRes = RoadwayPainter::paintRoadway((void*)(&ldwDataPack), frame.data, frame.cols, frame.rows);
		}
	}
	break;
	// 左+障碍物+车道线
	case ImageType::CalibLeft_LaneExt_Obstacle_OnlyDistance:
	case ImageType::CalibLeft_LaneExt_Obstacle_Details:
	case ImageType::CalibLeft_LaneExt_Obstacle_OnlyDistance_Nearest:
	case ImageType::CalibLeft_LaneExt_Obstacle_Details_Nearest:
	{
		SetObscaleParam(imgType);
		_camera->enableTasks(TaskId::DisplayTask | TaskId::ObstacleTask | TaskId::LaneTask);
		_camera->requestFrame(frameMonitor, FrameId::Compound | FrameId::LaneExt);
		frame = frameMonitor->getFrameMat(FrameId::Compound);

		LdwDataPack ldwDataPack = frameMonitor->getFrameLane(FrameId::LaneExt);
		if (ldwDataPack.softStatus >= 0) {
			bool roadwayRes = RoadwayPainter::paintRoadway((void*)(&ldwDataPack), frame.data, frame.cols, frame.rows);
		}
	}
	break;
	default:
		break;
	}
	return frame;
}
/// <summary>
/// 获取图像数据
/// </summary>
/// <param name="rows"></param>
/// <param name="cols"></param>
/// <param name="imgType"></param>
/// <param name="isYolo"></param>
/// <param name="ret_data"></param>
/// <returns></returns>
FrameResult* SmartEyeHandler::GetFrame(int rows, int cols, int imgType, bool isYolo, unsigned char* ret_data)
{
	FrameResult result;
	result.success = true;
	cv::Mat frame = GetFrame(imgType, &result);
	cout << "result.message --> " << result.message << endl;
	if (!result.success) {
		return &result;
	}
	if (!frame.empty()) {
		/*if (isYolo) {
			YoloDistinguish* yolo = YoloDistinguish::getInstance();
			yolo->TestingFrame(frame);
		}*/
		//cv::imshow("CalibRightCamera", frame);
		cv::Mat dst(rows, cols, frame.type());
		cv::resize(frame, dst, dst.size());
		memcpy(ret_data, dst.data, dst.rows * dst.cols * dst.channels());
	}
	else
	{
		result.success = false;
		result.message = "frame is empty";
		cout << "frame is empty" << endl;
	}

	result.frameId = frameMonitor->imageFrameDataPack->frameId;
	result.time = frameMonitor->imageFrameDataPack->time;
	result.index = frameMonitor->imageFrameDataPack->index;
	result.format = frameMonitor->imageFrameDataPack->format;
	result.width = frameMonitor->imageFrameDataPack->width;
	result.height = frameMonitor->imageFrameDataPack->height;
	result.speed = frameMonitor->imageFrameDataPack->speed;
	result.dataSize = frameMonitor->imageFrameDataPack->dataSize;

	return &result;
}

#pragma region 图像采集
/// <summary>
/// 获取项目根路径
/// </summary>
/// <returns></returns>
string SmartEyeHandler::GetRootPath()
{
	char* path = nullptr;
	path = _getcwd(nullptr, 1);
	puts(path);

	string str1(path);
	int pos = str1.find_last_of(os_pathsep, str1.length());
	std::string solutionDir = str1.substr(0, pos);  // 返回解决方案的路径
	return solutionDir;

	delete path;
	path = nullptr;
}
/// <summary>
/// 获取当前时间字符串
/// </summary>
/// <param name="hasMillsec">是否包含毫秒</param>
/// <returns></returns>
string SmartEyeHandler::GetCurrentDateStr(bool hasMillsec) {
	const char* format = !hasMillsec ? "%Y_%m_%d_%H_%M" : "%Y%m%d%H%M%S";
	time_t t1 = time(0);
	char ch1[64];
	strftime(ch1, sizeof(ch1), format, localtime(&t1)); //年-月-日 时-分
	std::string dateStr = ch1;

	if (hasMillsec) {
		system_clock::time_point time_point_now = system_clock::now(); // 获取当前时间点
		system_clock::duration duration_since_epoch = time_point_now.time_since_epoch(); // 从1970-01-01 00:00:00到当前时间点的时长
		time_t microseconds_since_epoch = duration_cast<microseconds>(duration_since_epoch).count(); // 将时长转换为微秒数
		time_t tm_millisec = microseconds_since_epoch / 1000 % 1000; // 当前时间的毫秒数
		dateStr.append(to_string(tm_millisec));
	}

	return dateStr;

	delete& t1;
}
/// <summary>
/// 创建文件夹
/// </summary>
/// <param name="szDir"></param>
/// <returns></returns>
int makeDirs(const char* szDir)
{
	if (NULL == szDir)
	{
		return -1;
	}
	if (-1 != _access(szDir, 0)) 
	{
		return 1;
	}

	int iRet = 0;
	std::string strDir = szDir;
	int index = strDir.find_last_of(os_pathsep);
	if (0 < index) //存在多级目录
	{
		strDir.erase(index, strDir.length() - index);

#ifdef __POSIX__
		if (-1 == access(strDir.c_str(), 0)) //若父目录不存在，则创建父目录
		{
			iRet = makeDirs(strDir.c_str()); //递归创建父目录
		}
#else //ISO
		if (-1 == _access(strDir.c_str(), 0)) //若父目录不存在，则创建父目录
		{
			iRet = makeDirs(strDir.c_str()); //递归创建父目录
		}
#endif // __POSIX__	
	}

	if (0 == iRet) //父目录创建成功
	{
#ifdef __POSIX__
		iRet = mkdir(szDir, 0755);
#else
		iRet = _mkdir(szDir);
#endif // __POSIX__
	}

	return iRet;
}
/// <summary>
/// mat保存为.dat文件
/// </summary>
/// <param name="mat"></param>
/// <param name="filePath"></param>
/// <returns></returns>
bool saveMatAsDat(cv::Mat mat, string filePath) {
	cout << "rows:" << mat.rows << endl;
	cout << "cols:" << mat.cols << endl;
	//打开文件
	ofstream outfile;
	outfile.open(filePath.c_str(), ios::binary);
	if (!outfile)
		return false;
	//写入二进制文件
	for (int r = 0; r < mat.rows; r++)
		outfile.write(reinterpret_cast<const char*>(mat.ptr(r)), mat.cols * mat.elemSize());
	outfile.close();
	return true;
	delete &outfile;
}
/// <summary>
/// Mat保存为raw
/// </summary>
/// <param name="mat"></param>
/// <param name="filePath"></param>
/// <returns></returns>
bool saveMatAsRaw(cv::Mat mat, string filePath) {
	// todo
}
/// <summary>
/// mat保存为jpg
/// </summary>
/// <param name="mat"></param>
/// <param name="filePath"></param>
/// <returns></returns>
bool saveMatAsJpg(cv::Mat mat, string filePath) {
	cv::imwrite(filePath, mat);
}
/// <summary>
/// 图像采集
/// </summary>
/// <param name="rows"></param>
/// <param name="cols"></param>
/// <param name="imgType"></param>
/// <returns></returns>
bool SmartEyeHandler::CollectFrame(int rows, int cols, int imgType)
{
	FrameResult result;
	result.success = true;
	cv::Mat frame = GetFrame(imgType, &result);
	if (!result.success) {
		return &result;
	}
	if (frame.empty())
	{
		result.success = false;
		result.message = "frame is empty";
		return false;
	}

	cv::Mat dst(rows, cols, frame.type());
	cv::resize(frame, dst, dst.size());

	string root = GetRootPath()
		.append(os_pathsep)
		.append("ADAS_Recorded_Images")
		.append(os_pathsep);
	string flodarDateStr = GetCurrentDateStr(false);
	root.append(flodarDateStr).append(os_pathsep);

	string fileNameDateStr = GetCurrentDateStr(true);
	RawImageFrame* imageData = frameMonitor->imageFrameDataPack;
	string filename = "";
	filename.append(fileNameDateStr)
		.append("_W")
		.append(to_string(imageData->width))
		.append("_Speed")
		.append(to_string(imageData->speed));

	switch (imgType)
	{
	case ImageType::CalibLeftCamera:
		root.append("left_images");
		filename.insert(0, "LRemap_");
		//filename.append(".raw");
		filename.append(".jpg");
		break;
	case ImageType::RightCamera:
		root.append("right_images");
		filename.insert(0, "RRemap_");
		//filename.append(".raw");
		filename.append(".jpg");
		break;
	case ImageType::Disparity:
		root.append("disparity_images");
		filename.insert(0, "disparity_");
		//filename.append(".raw");
		filename.append(".jpg");
		break;
	default:
		break;
	}
	int mkres = makeDirs(root.c_str());
	if (mkres == -1)
		return false;

	string filePath = root.append(os_pathsep).append(filename);
	/*if (imgType == ImageType::Disparity)
		return saveMatAsDat(frame, filePath);
	else
		return saveMatAsRaw(frame, filePath);*/
	saveMatAsJpg(dst, filePath);
}

#pragma endregion
