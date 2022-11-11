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
    bool showDetials, // 是否显示障碍物详细信息
         singleObs; // 是否仅绘制预警区域内最近障碍物

protected:
    void loadFrameData2Mat(const RawImageFrame* frameData, cv::Mat& dstMat);

private:
    std::mutex mMutex;
    std::condition_variable mFrameReadyCond;
    bool mFrameReadyFlag;

    cv::Mat mLeftMat; // 左图
    cv::Mat mRightMat; // 右图
    cv::Mat mDisparityMat; // 视差图
    cv::Mat mCompoundMat; // 障碍图
    LdwDataPack* mldwDataPack; // 车道线数据

    std::unique_ptr<float[]> mDisparityFloatData;
    std::unique_ptr<float[]> mDisparityDistanceZ;
    std::unique_ptr<unsigned char[]> mRgbBuffer;


};

#endif // CAMERAFRAMEMONITOR_H