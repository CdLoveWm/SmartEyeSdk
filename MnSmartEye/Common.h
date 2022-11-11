#pragma once
/// <summary>
/// 图像类型
/// </summary>
struct ImageType
{
    enum Enumeration {
        CalibLeftCamera = 1 << 0,  // 左相机校准图
        RightCamera = 1 << 1,  // 右相机原图
        Disparity = 1 << 2,  // 视差图

        CalibLeft_Obstacle_OnlyDistance = 1 << 3,  // 左校准图+仅显示障碍物距离
        CalibLeft_Obstacle_Details = 1 << 4,  // 左校准图+障碍物综合信息
        CalibLeft_Obstacle_OnlyDistance_Nearest = 1 << 5,  // 左校准图+仅显示障碍物距离+仅显示最近障碍物
        CalibLeft_Obstacle_Details_Nearest = 1 << 6,  // 左校准图+障碍物综合信息+仅显示最近障碍物

        CalibLeft_LaneExt = 1 << 7,  // 左校准图+车道线
        CalibLeft_LaneExt_Obstacle_OnlyDistance = 1 << 8,  // 左校准图+车道线+仅显示障碍物距离
        CalibLeft_LaneExt_Obstacle_Details = 1 << 9,  // 左校准图+车道线+障碍物综合信息
        CalibLeft_LaneExt_Obstacle_OnlyDistance_Nearest = 1 << 10,  // 左校准图+车道线+仅显示障碍物距离+仅显示最近障碍物
        CalibLeft_LaneExt_Obstacle_Details_Nearest = 1 << 11,  // 左校准图+车道线+障碍物综合信息+仅显示最近障碍物

    };
};

struct FrameResult
{

    uint16_t frameId;
    int64_t  time;
    uint16_t index;
    uint16_t format;
    uint16_t width;
    uint16_t height;
    uint32_t speed;
    uint32_t dataSize;
    uint16_t success;
    const char* message;
};
