#pragma once
#include <opencv2/core.hpp>
#include "yolo_v2_class.hpp"

class YoloDistinguish
{
public:
	std::vector<bbox_t> TestingFrame(cv::Mat frame);
	void show(int n);
	static YoloDistinguish* getInstance();
	Detector* initOrGetDetector();
private:
	YoloDistinguish() {}; // 构造私有化
	~YoloDistinguish() {};
	YoloDistinguish(const YoloDistinguish&);
	YoloDistinguish& operator=(const YoloDistinguish&);

	std::string prevSrc = "datas\\";
	std::string names_file = prevSrc + "traffic.names";
	std::string cfg_file = prevSrc + "traffic.cfg";
	std::string weights_file = prevSrc + "traffic.weights";

	void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names,
		int current_det_fps = -1, int current_cap_fps = -1);
	std::vector<std::string> objects_names_from_file(std::string const filename);
	std::vector<std::string> getObjNames();
};

