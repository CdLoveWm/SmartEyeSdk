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

#pragma region ��¶����
/// <summary>
/// ��ȡ�Ƿ�����
/// </summary>
/// <returns></returns>
extern "C" _declspec(dllexport)bool IsConnected()
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->IsCameraConnected();
}
/// <summary>
/// ��ʼ������
/// </summary>
/// <param name="addr">IP��ַ</param>
/// <returns></returns>
extern "C" _declspec(dllexport)bool InitConnect(char* addr)
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->InitCamera(addr);
}
/// <summary>
/// �ͷ���Դ���Ͽ�����
/// </summary>
/// <returns></returns>
extern "C" _declspec(dllexport)bool Release()
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->ReleaseCamera();
}
/// <summary>
/// ��ʼ��YOLO
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
/// ���Է���ͼƬ����2
/// </summary>
/// <param name="rows">��</param>
/// <param name="cols">��</param>
/// <param name="frameId">ͼ�����ݱ�ʶ</param>
/// <param name="isTesting">�Ƿ���yolo���</param>
/// <param name="ret_data"></param>
extern "C" _declspec(dllexport)FrameResult* GetImage2(int rows, int cols, int imgType, bool isYolo, unsigned char* ret_data)
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->GetFrame(rows, cols, imgType, isYolo, ret_data);
}
/// <summary>
/// ���Է���ͼƬ����2
/// </summary>
/// <param name="rows">��</param>
/// <param name="cols">��</param>
/// <param name="frameId">ͼ�����ݱ�ʶ</param>
/// <param name="isTesting">�Ƿ���yolo���</param>
/// <param name="ret_data"></param>
extern "C" _declspec(dllexport)bool CollectFrame(int rows, int cols, int imgType)
{
    hanlder = SmartEyeHandler::GetInstance();
    return hanlder->CollectFrame(rows, cols, imgType);
}


#pragma endregion
