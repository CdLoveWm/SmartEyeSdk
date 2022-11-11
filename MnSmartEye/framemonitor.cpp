#include "framemonitor.h"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "satpext.h"
#include "frameid.h"
#include "frameformat.h"
#include "disparityconvertor.h"
#include "yuv2rgb.h" 
#include "frameext.h"
#include "roadwaypainter.h"
#include "obstacleData.h"
#include <obstaclepainter.h>

FrameMonitor::FrameMonitor()
    : mFrameReadyFlag(false),
    mDisparityFloatData(new float[1280 * 720]),
    mDisparityDistanceZ(new float[1280 * 720]),
    mRgbBuffer(new unsigned char[1280 * 720 * 3])
{
    // support gray or rgb
    mLeftMat.create(720 , 1280 , CV_8UC3);
    mDisparityMat.create(720 / 2, 1280 / 2, CV_8UC3); 
    mldwDataPack = (LdwDataPack*)malloc(sizeof(LdwDataPack));
    outputObstacles = (OutputObstacles*)malloc(sizeof(OutputObstacles));
    mCompoundMat.create(720, 1280, CV_8UC3);
}
FrameMonitor::~FrameMonitor()
{ 
    delete(mldwDataPack);
    delete(outputObstacles);
}
void FrameMonitor::handleRawFrame(const RawImageFrame* rawFrame)
{
    processFrame(rawFrame);
}

void FrameMonitor::processFrame(const RawImageFrame* rawFrame)
{
    switch (rawFrame->frameId) {
    case FrameId::Disparity: // 视差数据
    {
        // only for FrameFormat::Disparity16, bitNum = 5
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mDisparityMat);

        //        std::cout << "update disparity mat" << std::endl;
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
    break;
    case FrameId::CalibLeftCamera: //保留左图，去除右图输出能力 左相机校准图
    { 
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mLeftMat);
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
    break;
    case FrameId::LaneExt:
    {
        std::cout << "33333\n";
        FrameDataExtHead* header = reinterpret_cast<FrameDataExtHead*>((char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame));
        memcpy((void*)(mldwDataPack), (void*)(header->data), sizeof(LdwDataPack));
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
    break;
    case FrameId::Obstacle:
    {
        char* obstacleParam = (char*)rawFrame + sizeof(RawImageFrame);
        int blockNum = reinterpret_cast<int*>(obstacleParam)[0];
        printf("block num: %d\n", blockNum);
        if (blockNum > 0) {
            OutputObstacles* pOutputObstacles = reinterpret_cast<OutputObstacles*> (reinterpret_cast<int*>(obstacleParam) + 2);
            std::cout << "timestamp: " << time << std::endl;
            printf("car speed: %3f\n", pOutputObstacles[0].currentSpeed);
            printf("frame rate: %3f\n", pOutputObstacles[0].frameRate);
            for (int i = 0; i < blockNum; i++)
            {
                float distance = pOutputObstacles[i].avgDistanceZ;
                printf("averageDistanceZ: %.2fm\n", distance);
                switch (pOutputObstacles[i].classLabel) {
                case 4:
                {
                    printf("left continuous obstacle\n");
                }
                break;
                case 5:
                {
                    printf("right continuous obstacle\n");
                }
                break;
                default:
                {
                    printf("classify obstacle for front collision: ");
                    if (pOutputObstacles[i].stateLabel == 1)
                    {
                        printf("nearest obstacle in warning area\n");
                    }
                    else if (pOutputObstacles[i].stateLabel == 2) {
                        printf("other obstacles in warning area\n");
                    }
                    else if (pOutputObstacles[i].stateLabel == 3) {
                        printf("obstacles out of warning area\n");
                    }
                    printf("tracking obstacle ID : %d\n", pOutputObstacles[i].trackId);
                    printf("track frame numbers: %d\n", pOutputObstacles[i].trackFrameNum);
                    printf("obstacle width: %.2fm\n", pOutputObstacles[i].fuzzy3DWidth);
                    printf("obstacle height: %.2fm\n", (pOutputObstacles[i].real3DUpY - pOutputObstacles[i].real3DLowY));
                    printf("SpeedZ: %.2fm/s\n", pOutputObstacles[i].fuzzyRelativeSpeedZ);
                    printf("SpeedX: %.2fm/s\n", pOutputObstacles[i].fuzzyRelativeSpeedCenterX);
                    printf("TTCZ: %.2fs\n", pOutputObstacles[i].fuzzyCollisionTimeZ);
                    printf("DistZ: %.2fm\n", pOutputObstacles[i].fuzzyRelativeDistanceZ);
                    printf("obstacle type: ");
                    switch (pOutputObstacles[i].obstacleType)
                    {
                    case 1:
                        printf("vehicle\n");
                        break;
                    case 2:
                        printf("pedestrian\n");
                        break;
                    default:
                        printf("others\n");
                        break;
                    }

                }
                break;
                }
                printf("\n");
            }
        }

    }
    break;
    case FrameId::Compound:
    {

     //   FrameDataExtHead* header = reinterpret_cast<FrameDataExtHead*>((char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame));
     //   int blockNum = ((int*)header->data)[0];
     //   /*OutputObstacles* obsDataPack = (OutputObstacles*)(((int*)header->data) + 2);
     //   memcpy((void*)(outputObstacles), (void*)(header->data), sizeof(OutputObstacles));*/

     //   std::cout << "blockNum is: " << blockNum << std::endl;
     ///*   static unsigned char* rgbBuf = new unsigned char[rawFrame->width * rawFrame->height * 3];
     //   RoadwayPainter::imageGrayToRGB((unsigned char*)((char*)rawFrame + sizeof(RawImageFrame)), rgbBuf, rawFrame->width, rawFrame->height);
     //   ObstaclePainter::paintObstacle(header->data, rgbBuf, rawFrame->width, rawFrame->height, true, false);

     //   cv::Mat frame (rawFrame->width, rawFrame->height, CV_8UC3, rgbBuf);
     //   cv::imshow("frame11", frame);*/

        int width = rawFrame->width;
        int height = rawFrame->height;
        FrameDataExtHead* header = reinterpret_cast<FrameDataExtHead*>((char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame));
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mCompoundMat);
        ObstaclePainter::paintObstacle(header->data, mCompoundMat.data, width, height, true, false);
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();

    }
    break;
    }
}

void FrameMonitor::waitForFrames()
{
    std::unique_lock<std::mutex> lock(mMutex);
    mFrameReadyCond.wait_for(lock, std::chrono::milliseconds(240), [this] {
        return mFrameReadyFlag;
        });

    mFrameReadyFlag = false;
}

cv::Mat FrameMonitor::getFrameMat(int frameId)
{
    std::lock_guard<std::mutex> lock(mMutex);

    switch (frameId) {
    case FrameId::Disparity:
        return mDisparityMat.clone();
    case FrameId::CalibLeftCamera: 
        return mLeftMat.clone();
    case FrameId::Compound:
        return mCompoundMat.clone();
    default:
        return mLeftMat.clone();
    }
}



LdwDataPack FrameMonitor::getFrameLane(int frameId)
{
    LdwDataPack ldwDataPack;
    std::lock_guard<std::mutex> lock(mMutex);

    switch (frameId) {
    case FrameId::LaneExt:
        memcpy(&ldwDataPack, mldwDataPack, sizeof(LdwDataPack));
        return ldwDataPack;
    }

}
OutputObstacles FrameMonitor::getFrameObstacles(int frameId)
{
    OutputObstacles templdwDataPack;
    std::lock_guard<std::mutex> lock(mMutex);

    switch (frameId) {
    case FrameId::Compound:
        memcpy(&templdwDataPack, outputObstacles, sizeof(OutputObstacles));
        return templdwDataPack;
    }

}


void FrameMonitor::loadFrameData2Mat(const RawImageFrame* frameData, cv::Mat& dstMat)
{
    int width = frameData->width;
    int height = frameData->height;
    const unsigned char* imageData = frameData->image;

    switch (frameData->format) {
    case FrameFormat::Disparity16:
    case FrameFormat::DisparityDens16:
    {
        //        DisparityConvertor::convertDisparity2FloatFormat(imageData, width, height, 5, mDisparityFloatData.get());
        //        DisparityConvertor::convertDisparity2RGB(mDisparityFloatData.get(), width, height, 0, 45, mRgbBuffer.get());
        //        cv::Mat dispMat(height, width, CV_8UC3, mRgbBuffer.get());
        //        cv::resize(dispMat, dstMat, dstMat.size());

        cv::Mat dispMat(height, width, CV_16U, (void*)imageData);
        cv::resize(dispMat, dstMat, dstMat.size());
        cv::normalize(dstMat, dstMat, 0, 255, cv::NORM_MINMAX, CV_8U);
    }
    break;
    case FrameFormat::Gray:
    {
        cv::Mat grayMat(height, width, CV_8UC1, (void*)imageData);
        cv::resize(grayMat, dstMat, dstMat.size());
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
