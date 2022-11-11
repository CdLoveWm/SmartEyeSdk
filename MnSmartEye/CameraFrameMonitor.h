#pragma once

#ifndef CAMERAFRAMEMONITOR_H
#define CAMERAFRAMEMONITOR_H

#include <thread>
#include <mutex>
#include <memory>
#include <condition_variable>
#include <opencv2/core.hpp>
#include "camerahandler.h"
#include "LdwDataInterface.h"

class CameraFrameMonitor : public CameraHandler
{
public:
    CameraFrameMonitor();
    virtual ~CameraFrameMonitor();
    void handleRawFrame(const RawImageFrame* rawFrame);
    void processFrame(const RawImageFrame* rawFrame);
    void waitForFrames();
    cv::Mat getFrameMat(int frameId);
    LdwDataPack getFrameLane(int frameId);
    RawImageFrame* imageFrameDataPack;
    void Releae();
    bool showDetials, // �Ƿ���ʾ�ϰ�����ϸ��Ϣ
         singleObs; // �Ƿ������Ԥ������������ϰ���

protected:
    void loadFrameData2Mat(const RawImageFrame* frameData, cv::Mat& dstMat);

private:
    std::mutex mMutex;
    std::condition_variable mFrameReadyCond;
    bool mFrameReadyFlag;

    cv::Mat mLeftMat; // ��ͼ
    cv::Mat mRightMat; // ��ͼ
    cv::Mat mDisparityMat; // �Ӳ�ͼ
    cv::Mat mCompoundMat; // �ϰ�ͼ
    LdwDataPack* mldwDataPack; // ����������

    std::unique_ptr<float[]> mDisparityFloatData;
    std::unique_ptr<float[]> mDisparityDistanceZ;
    std::unique_ptr<unsigned char[]> mRgbBuffer;


};

#endif // CAMERAFRAMEMONITOR_H