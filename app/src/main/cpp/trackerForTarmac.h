//
// Created by Ifand on 2019/11/25.
//

#ifndef TRACKERFORTARMAC_H
#define TRACKERFORTARMAC_H

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <android/log.h>

/**
 * 记录不同高度的HSV值
 */
typedef struct ColorRange
{
    int HeightMin;     // 最小高度
    int HeightMax;     // 最大高度
    int Hmin;          // H最小值
    int Hmax;          // H最大值
    int Smin;          // S最小值
    int Smax;          // S最大值
    int Vmin;          // V最小值
    int Vmax;          // V最大值
} ColorRange;

/**
 * 配置信息.
 *
 * \note 检测结果包括检测过程中录像和包含特定时间检测结果的图片.
 */
typedef struct TrackerConfig
{
    float trackMaxTime;        // 跟踪时长
    int minTargetArea;         // 最小目标区域
    std::string resultDir;     // 检测结果保存路径
    float searchMinHeight;     // 无人机搜索飞行的最小高度
    float searchMaxHeight;     // 无人机搜索飞行的最大高度
    float groundAltitude;      // 地面的海拔高度
    float yawAngle;            // 无人机的飞行方向
    ColorRange ColorRanges[3]; //  不同高度对应的颜色范围
};

using namespace cv;

bool FindTargetInFrame(Mat srcFrame, int nHeight, Rect2d& matchRect, int minTargetArea);

#endif //TRACKER_H
