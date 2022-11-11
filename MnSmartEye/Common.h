#pragma once
/// <summary>
/// ͼ������
/// </summary>
struct ImageType
{
    enum Enumeration {
        CalibLeftCamera = 1 << 0,  // �����У׼ͼ
        RightCamera = 1 << 1,  // �����ԭͼ
        Disparity = 1 << 2,  // �Ӳ�ͼ

        CalibLeft_Obstacle_OnlyDistance = 1 << 3,  // ��У׼ͼ+����ʾ�ϰ������
        CalibLeft_Obstacle_Details = 1 << 4,  // ��У׼ͼ+�ϰ����ۺ���Ϣ
        CalibLeft_Obstacle_OnlyDistance_Nearest = 1 << 5,  // ��У׼ͼ+����ʾ�ϰ������+����ʾ����ϰ���
        CalibLeft_Obstacle_Details_Nearest = 1 << 6,  // ��У׼ͼ+�ϰ����ۺ���Ϣ+����ʾ����ϰ���

        CalibLeft_LaneExt = 1 << 7,  // ��У׼ͼ+������
        CalibLeft_LaneExt_Obstacle_OnlyDistance = 1 << 8,  // ��У׼ͼ+������+����ʾ�ϰ������
        CalibLeft_LaneExt_Obstacle_Details = 1 << 9,  // ��У׼ͼ+������+�ϰ����ۺ���Ϣ
        CalibLeft_LaneExt_Obstacle_OnlyDistance_Nearest = 1 << 10,  // ��У׼ͼ+������+����ʾ�ϰ������+����ʾ����ϰ���
        CalibLeft_LaneExt_Obstacle_Details_Nearest = 1 << 11,  // ��У׼ͼ+������+�ϰ����ۺ���Ϣ+����ʾ����ϰ���

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
