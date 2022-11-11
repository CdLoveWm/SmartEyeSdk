#include "CameraFrameMonitor.h"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "satpext.h"
#include "frameid.h"
#include "frameformat.h"
#include "disparityconvertor.h"
#include "ObstaclePainter.h"
#include "yuv2rgb.h"
#include "frameext.h"
#include "roadwaypainter.h"


CameraFrameMonitor::CameraFrameMonitor()
    : mFrameReadyFlag(false),
    mDisparityFloatData(new float[1280 * 720]),
    mDisparityDistanceZ(new float[1280 * 720]),
    mRgbBuffer(new unsigned char[1280 * 720 * 3]),
    showDetials(false),
    singleObs(false)
{
    mLeftMat.create(720 / 2, 1280 / 2, CV_8UC3);
    mRightMat.create(720 / 2, 1280 / 2, CV_8UC3);
    //mDisparityMat.create(720 / 2, 1280 / 2, CV_8UC1); // 获取单通道灰度视差的时候这样定义
    mDisparityMat.create(720 / 2, 1280 / 2, CV_8UC3);
    mCompoundMat.create(720, 1280, CV_8UC3);
    mldwDataPack = (LdwDataPack*)malloc(sizeof(LdwDataPack));
    imageFrameDataPack = new RawImageFrame();
}

CameraFrameMonitor::~CameraFrameMonitor()
{
}
/// <summary>
/// 相机SDK执行回调
/// </summary>
/// <param name="rawFrame"></param>
void CameraFrameMonitor::handleRawFrame(const RawImageFrame* rawFrame)
{
    processFrame(rawFrame);
}
/// <summary>
/// 图像处理函数
/// </summary>
/// <param name="rawFrame"></param>
void CameraFrameMonitor::processFrame(const RawImageFrame* rawFrame)
{
    {
        imageFrameDataPack->frameId = rawFrame->frameId;
        imageFrameDataPack->time = rawFrame->time;
        imageFrameDataPack->index = rawFrame->index;
        imageFrameDataPack->format = rawFrame->format;
        imageFrameDataPack->width = rawFrame->width;
        imageFrameDataPack->height = rawFrame->height;
        imageFrameDataPack->speed = rawFrame->speed;
        imageFrameDataPack->dataSize = rawFrame->dataSize;
    }
    int width = rawFrame->width;
    int height = rawFrame->height;
    FrameDataExtHead* header = reinterpret_cast<FrameDataExtHead*>((char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame));

    switch (rawFrame->frameId) {
    case FrameId::Disparity: // 视差数据
    {
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mDisparityMat);
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
    break;
    case FrameId::CalibLeftCamera: // 左相机校准图
    case FrameId::LeftCamera: // 左相机原图
    {
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mLeftMat);
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
    break;
    case FrameId::CalibRightCamera: // 右相机校准图
    case FrameId::RightCamera: // 右相机原图
    {
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mRightMat);
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
    break;
    case FrameId::LaneExt: // 左校准图与车道线数据的复合数据
    {
        memcpy((void*)(mldwDataPack), (void*)(header->data), sizeof(LdwDataPack));
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
    break;
    case FrameId::Compound: // 左校准图与障碍物数据的复合数据
    {
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mCompoundMat);
        ObstaclePainter::paintObstacle(header->data, mCompoundMat.data, width, height, showDetials, singleObs);
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
    break;
    }
}

void CameraFrameMonitor::waitForFrames()
{
    std::unique_lock<std::mutex> lock(mMutex);
    mFrameReadyCond.wait_for(lock, std::chrono::milliseconds(240), [this] {
        return mFrameReadyFlag;
        });

    mFrameReadyFlag = false;
}

cv::Mat CameraFrameMonitor::getFrameMat(int frameId)
{
    std::lock_guard<std::mutex> lock(mMutex);

    switch (frameId) {
    case FrameId::RightCamera:
        return mRightMat.clone();
    case FrameId::Disparity:
        return mDisparityMat.clone();
    case FrameId::Compound:
        return mCompoundMat.clone();
    default:
        return mLeftMat.clone();
    }
}

LdwDataPack CameraFrameMonitor::getFrameLane(int frameId)
{
    LdwDataPack ldwDataPack;
    std::lock_guard<std::mutex> lock(mMutex);

    switch (frameId) {
    case FrameId::LaneExt:
        memcpy(&ldwDataPack, mldwDataPack, sizeof(LdwDataPack));
        return ldwDataPack;
    }

}
/// <summary>
/// 释放资源
/// </summary>
void CameraFrameMonitor::Releae()
{
    delete &mLeftMat;
    delete& mRightMat;
    delete& mDisparityMat;
    delete& mCompoundMat;
    delete mldwDataPack;
    delete& imageFrameDataPack;
}

void CameraFrameMonitor::loadFrameData2Mat(const RawImageFrame* frameData, cv::Mat& dstMat)
{
    int width = frameData->width;
    int height = frameData->height;
    const unsigned char* imageData = frameData->image;

    switch (frameData->format) {
    case FrameFormat::Disparity16:
    {
        DisparityConvertor::convertDisparity2FloatFormat(imageData, width, height, 5, mDisparityFloatData.get());
        DisparityConvertor::convertDisparity2RGB(mDisparityFloatData.get(), width, height, 0, 45, mRgbBuffer.get());
        cv::Mat dispMat(height, width, CV_8UC3, mRgbBuffer.get());
        cv::resize(dispMat, dstMat, dstMat.size());
        cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);

        /*cv::Mat dispMat(height, width, CV_16U, (void*)imageData);
        cv::resize(dispMat, dstMat, dstMat.size());
        cv::normalize(dstMat, dstMat, 0, 255, cv::NORM_MINMAX, CV_8U);*/
    }
    break;
    case FrameFormat::Gray:
    {
        cv::Mat grayMat(height, width, CV_8UC1, (void*)imageData);
        cv::resize(grayMat, dstMat, dstMat.size());
        cv::cvtColor(dstMat, dstMat, CV_GRAY2RGB);
    }
    break;
    case FrameFormat::YUV422:
    {
        YuvToRGB::YCbYCr2Rgb(imageData, (char*)mRgbBuffer.get(), width, height);
        cv::Mat yuv422Mat(height, width, CV_8UC3, mRgbBuffer.get());
        cv::resize(yuv422Mat, dstMat, dstMat.size());
        cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);
    }
    break;
    case FrameFormat::YUV422Plannar:
    {
        YuvToRGB::YCbYCrPlannar2Rgb(imageData, (char*)mRgbBuffer.get(), width, height);
        cv::Mat yuv422PlannarMat(height, width, CV_8UC3, mRgbBuffer.get());
        cv::resize(yuv422PlannarMat, dstMat, dstMat.size());
        cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);
    }
    break;
    }
}
