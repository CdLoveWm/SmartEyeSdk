#include <iostream>

#ifdef _WIN32
#define OPENCV
#define GPU
#endif

#include "Distinguish.h"
#include "yolo_v2_class.hpp" //引用动态链接库中的头文件
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"


#pragma comment(lib, "opencv_world341.lib") //引入OpenCV链接库
#pragma comment(lib, "yolo_cpp_dll.lib") //引入YOLO动态链接库



/// <summary>
/// 获取实例
/// </summary>
/// <returns></returns>
YoloDistinguish* YoloDistinguish::getInstance()
{
	static YoloDistinguish instance;
	return &instance;
}
/// <summary>
/// 获取Detector
/// </summary>
/// <returns></returns>
Detector* YoloDistinguish::initOrGetDetector()
{
	static Detector detector = Detector(cfg_file, weights_file); //初始化检测器
	std::cout << "----------------------------->" << &detector << std::endl;
	return &detector;
}
/// <summary>
/// 获取分类对象名称
/// </summary>
/// <param name="filename"></param>
/// <returns></returns>
std::vector<std::string> YoloDistinguish::objects_names_from_file(std::string const filename) {
	std::cout << "*****************objects_names_from_file************************" << std::endl;
	std::ifstream file(filename);
	std::vector<std::string> file_lines;
	if (!file.is_open()) return file_lines;
	for (std::string line; getline(file, line);) file_lines.push_back(line);
	return file_lines;
}
/// <summary>
/// 获取标注对象名称数组
/// </summary>
/// <returns></returns>
std::vector<std::string> YoloDistinguish::getObjNames()
{
	static std::vector<std::string> obj_names = objects_names_from_file(names_file);
	return obj_names;
}
/// <summary>
/// 标记
/// </summary>
/// <param name="mat_img"></param>
/// <param name="result_vec"></param>
/// <param name="obj_names"></param>
/// <param name="current_det_fps"></param>
/// <param name="current_cap_fps"></param>
void YoloDistinguish::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names,
	int current_det_fps, int current_cap_fps)
{
	int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

	for (auto& i : result_vec) {
		cv::Scalar color = obj_id_to_color(i.obj_id);
		cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
		if (obj_names.size() > i.obj_id) {
			std::string obj_name = obj_names[i.obj_id];
			if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
			cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
			int const max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
			cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 30, 0)),
				cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)),
				color, CV_FILLED, 8, 0);
			putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
		}
	}
	if (current_det_fps >= 0 && current_cap_fps >= 0) {
		std::string fps_str = "FPS detection: " + std::to_string(current_det_fps) + "   FPS capture: " + std::to_string(current_cap_fps);
		putText(mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
	}
}

/// <summary>
/// 图像检测
/// </summary>
/// <param name="frame"></param>
/// <returns></returns>
std::vector<bbox_t> YoloDistinguish::TestingFrame(cv::Mat frame) {
	Detector* detector = initOrGetDetector();
	std::vector<bbox_t> result_vec = detector->detect(frame);
	draw_boxes(frame, result_vec, getObjNames());
	return result_vec;
}

/// <summary>
/// 测试
/// </summary>
/// <param name="n"></param>
void YoloDistinguish::show(int n) {
	cv::VideoCapture capture;
	capture.open("F:\\Project\\2022\\WuDaProjects\\YOLO\\yoloSamples\\yolodllcalltest\\yolodllcall\\test\\dog.jpg");
	if (!capture.isOpened())
	{
		printf("文件打开失败");
	}
	cv::Mat frame;
	capture >> frame;
	TestingFrame(frame);

	cv::namedWindow("test" + n, CV_WINDOW_NORMAL);
	cv::imshow("test" + n, frame);
}