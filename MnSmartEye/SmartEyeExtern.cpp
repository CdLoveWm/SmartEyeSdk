#include <iostream>
#include <opencv2/highgui.hpp>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include <opencv2/imgproc.hpp>
#include "SmartEyeExtern.h"
#include "StereoCamera.h"
#include "protocol.h"
//#include "Distinguish.h"
#include "SmartEyeHandler.h"
#include "Common.h"
using namespace std;

SmartEyeHandler* hanlder;

#pragma region 暴露方法
/// <summary>
/// 获取是否连接
/// </summary>
/// <returns></returns>
extern "C" _declspec(dllexport)bool IsConnected()
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->IsCameraConnected();
}
/// <summary>
/// 初始化连接
/// </summary>
/// <param name="addr">IP地址</param>
/// <returns></returns>
extern "C" _declspec(dllexport)bool InitConnect(char* addr)
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->InitCamera(addr);
}
/// <summary>
/// 释放资源，断开连接
/// </summary>
/// <returns></returns>
extern "C" _declspec(dllexport)bool Release()
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->ReleaseCamera();
}
/// <summary>
/// 初始化YOLO
/// </summary>
/// <returns></returns>
extern "C" _declspec(dllexport)bool InitYolo()
{
    cout << "++++++++++++++++++InitYolo++++++++++++++++++" << endl;
    /*YoloDistinguish* yolo = YoloDistinguish::getInstance();
    yolo->initOrGetDetector();*/
    return true;
}
/// <summary>
/// 测试返回图片数据2
/// </summary>
/// <param name="rows">高</param>
/// <param name="cols">宽</param>
/// <param name="frameId">图像数据标识</param>
/// <param name="isTesting">是否开启yolo检测</param>
/// <param name="ret_data"></param>
extern "C" _declspec(dllexport)FrameResult* GetImage2(int rows, int cols, int imgType, bool isYolo, unsigned char* ret_data)
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->GetFrame(rows, cols, imgType, isYolo, ret_data);
}
/// <summary>
/// 测试返回图片数据2
/// </summary>
/// <param name="rows">高</param>
/// <param name="cols">宽</param>
/// <param name="frameId">图像数据标识</param>
/// <param name="isTesting">是否开启yolo检测</param>
/// <param name="ret_data"></param>
extern "C" _declspec(dllexport)bool CollectFrame(int rows, int cols, int imgType)
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->CollectFrame(rows, cols, imgType);
}


#pragma endregion
