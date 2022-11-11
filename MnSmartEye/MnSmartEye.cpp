// SmartEyeFrame.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
#include <iostream>
#include <opencv2/highgui.hpp>

#include "stereocamera.h"
#include "framemonitor.h"
#include "frameid.h"
#include "taskiddef.h"
#include "SmartEyeHandler.h"
#include "Common.h";
#include "windows.h"
#include "CameraFrameMonitor.h"
#include <opencv2/imgproc.hpp>
#include "SystemParam.h"
#include "mycamerahandler.h"
#include <chrono>
#include <roadwaypainter.h>
#include <ctime>
#include "LdwDataInterface.h"


using namespace std::chrono;

#pragma region 测试代码
/// <summary>
/// 测试图像返回
/// </summary>
void TestFrame() {
    StereoCamera* camera = StereoCamera::connect("192.168.1.251");
    std::unique_ptr<FrameMonitor> frameMonitor(new FrameMonitor);

    camera->enableTasks(TaskId::DisplayTask);
    camera->requestFrame(frameMonitor.get(), FrameId::CalibLeftCamera | FrameId::Disparity);

    std::function<int()> draw_func = [&frameMonitor]() -> int {
        cv::Mat leftFrame = frameMonitor->getFrameMat(FrameId::CalibLeftCamera);
        cv::Mat disparity = frameMonitor->getFrameMat(FrameId::Disparity);
        if (!leftFrame.empty()) {
            cv::imshow("Left", leftFrame);
        }

        if (!disparity.empty()) {
            cv::imshow("Disparity", disparity);
        }

        return cv::waitKey(80);
    };

    camera->invokeInLoopThread([] {
        cv::namedWindow("Left");
        cv::namedWindow("Disparity");
        });

    // 循环调用
    while (true) {
        frameMonitor->waitForFrames();  // wait for frames ready

        int key = 0;
        camera->invokeInLoopThread([&key, &draw_func] {
            key = draw_func();
            });

        if (key == 27) {
            // press Esc to close
            break;
        }
    }

    camera->invokeInLoopThread([] {
        cv::destroyAllWindows();
        });
}


void TestFrame3() {
    std::cout << "aaaaaaaa" << std::endl;
    SmartEyeHandler* hanlder = SmartEyeHandler::GetInstance();
    hanlder->InitCamera("192.168.1.251");
    static unsigned char* rgbBuf = new unsigned char[773 * 706 * 3];
    while (true)
    {
        hanlder->GetFrame(706, 773, ImageType::CalibLeft_LaneExt, false, rgbBuf);
        cv::Mat frame(706, 773, CV_8UC3, rgbBuf);
        cv::imshow("frame", frame);
        cv::waitKey(80);
    }
   
}
void GrayMaptoColor()
{
    const char* imagename = "C:\\Users\\dong.chen\\Pictures\\giraffe.jpg";
    //产生灰度图
    cv::Mat img = cv::imread(imagename);
    cv::Mat gray, color;
    cv::cvtColor(img, gray, CV_RGB2GRAY);
    //灰度彩色映射变换
    double vmin, vmax, alpha;
    minMaxLoc(gray, &vmin, &vmax);
    alpha = 255.0 / (vmax - vmin);
    gray.convertTo(gray, CV_8U, alpha, -vmin * alpha);
    applyColorMap(gray, color, CV_GRAY2RGB);


    imshow("image", img); //显示图像    
    imshow("gray", gray);
    imshow("color", color);
    cv::waitKey();

}

void Test5() {
    StereoCamera* camera = StereoCamera::connect("192.168.1.251");
    std::unique_ptr<FrameMonitor> frameMonitor(new FrameMonitor);

    camera->enableTasks(TaskId::DisplayTask | TaskId::LaneTask);
    camera->requestFrame(frameMonitor.get(), FrameId::CalibLeftCamera | FrameId::LaneExt);

    std::function<int()> draw_func = [&frameMonitor]() -> int {

        cv::Mat leftMat = frameMonitor->getFrameMat(FrameId::CalibLeftCamera);
        LdwDataPack ldwDataPack = frameMonitor->getFrameLane(FrameId::LaneExt);//获取车道线数据包

        RoadwayPainter::paintRoadway((void*)(&ldwDataPack), leftMat.data, 640, 360);//叠加左图和车道线信息

        cv::imshow("LaneExt", leftMat);//绘制叠加后的图像

        return cv::waitKey(80);

    };

    camera->invokeInLoopThread([] {
        cv::namedWindow("LaneExt");
        });

    // main thread loop for drawing images
    while (true) {
        frameMonitor->waitForFrames();  // wait for frames ready

        int key = 0;
        camera->invokeInLoopThread([&key, &draw_func] {
            key = draw_func();
            });

        if (key == 27) {
            // press Esc to close
            break;
        }
    }

    camera->invokeInLoopThread([] {
        cv::destroyAllWindows();
        });

}


void Test6() {
    StereoCamera* camera = StereoCamera::connect("192.168.1.251");
    std::unique_ptr<FrameMonitor> frameMonitor(new FrameMonitor);

    camera->enableTasks(TaskId::DisplayTask | TaskId::ObstacleTask);
    camera->requestFrame(frameMonitor.get(), FrameId::Compound);

    std::function<int()> draw_func = [&frameMonitor]() -> int {

        //cv::Mat leftMat = frameMonitor->getFrameMat(FrameId::Compound);
        //OutputObstacles outputObstacles = frameMonitor->getFrameObstacles(FrameId::Compound);//获取障碍物数据包

        //ObstaclePainter::paintObstacle((void*)(&outputObstacles), leftMat.data, 640, 360,true,true);//叠加左图和障碍物信息
        //
        //cv::imshow("Compound", leftMat);//绘制叠加后的图像

        //return cv::waitKey(80);
        cv::Mat compoundMat = frameMonitor->getFrameMat(FrameId::Compound);
        cv::imshow("Compound", compoundMat);
        return cv::waitKey(80);

    };

    camera->invokeInLoopThread([] {
        cv::namedWindow("Compound");
        });

    // main thread loop for drawing images
    while (true) {
        frameMonitor->waitForFrames();  // wait for frames ready

        int key = 0;
        camera->invokeInLoopThread([&key, &draw_func] {
            key = draw_func();
            });

        if (key == 27) {
            // press Esc to close
            break;
        }
    }

    camera->invokeInLoopThread([] {
        cv::destroyAllWindows();
        });


  
}
#pragma endregion

int main()
{
    TestFrame3();
    //Test5();
    /*SmartEyeHandler* hanlder = SmartEyeHandler::GetInstance();
    hanlder->InitCamera("192.168.1.251");
    while (true)
    {
        hanlder->CollectFrame(730, 700, ImageType::Disparity);
        hanlder->CollectFrame(730, 700, ImageType::CalibLeftCamera);
        hanlder->CollectFrame(730, 700, ImageType::CalibRightCamera);
        Sleep(800);
    }*/

    //time_t t1 = time(0);
    //char ch1[64];
    //strftime(ch1, sizeof(ch1), "%Y_%m_%d_%H_%M_%S_%M", localtime(&t1)); //年-月-日 时-分
    //std::string showText1 = ch1;


    //system_clock::time_point time_point_now = system_clock::now(); // 获取当前时间点
    //system_clock::duration duration_since_epoch = time_point_now.time_since_epoch(); // 从1970-01-01 00:00:00到当前时间点的时长
    //time_t microseconds_since_epoch = duration_cast<microseconds>(duration_since_epoch).count(); // 将时长转换为微秒数
    //time_t tm_millisec = microseconds_since_epoch / 1000 % 1000; // 当前时间的毫秒数

   /* SmartEyeHandler* hanlder = SmartEyeHandler::GetInstance();
    string dateStr = hanlder->GetCurrentDateStr(false);
    cout << "dateStr--> " << dateStr << "\n";*/

    return 0;
}


