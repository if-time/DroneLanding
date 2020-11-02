//
// Created by Ifand on 2019/11/29.
//

#include "trackerForTarmac.h"

struct TrackerConfig tracker_config = {
        .minTargetArea=64,
        .trackMaxTime=60,
        .searchMinHeight=30,
        .searchMaxHeight=50,
        .yawAngle=180,

        .ColorRanges[0].HeightMin = 0,
        .ColorRanges[0].HeightMax =10,
        .ColorRanges[0].Hmin=0,
        .ColorRanges[0].Hmax=76,
        .ColorRanges[0].Smin=118,
        .ColorRanges[0].Smax=255,
        .ColorRanges[0].Vmin=59,
        .ColorRanges[0].Vmax=124,

        .ColorRanges[1].HeightMin = 10,
        .ColorRanges[1].HeightMax=20,
        .ColorRanges[1].Hmin=50,
        .ColorRanges[1].Hmax=76,
        .ColorRanges[1].Smin=50,
        .ColorRanges[1].Smax=161,
        .ColorRanges[1].Vmin=155,
        .ColorRanges[1].Vmax=215,

        .ColorRanges[2].HeightMin = 20,
        .ColorRanges[2].HeightMax=50,
        .ColorRanges[2].Hmin=49,
        .ColorRanges[2].Hmax=81,
        .ColorRanges[2].Smin=4,
        .ColorRanges[2].Smax=96,
        .ColorRanges[2].Vmin=146,
        .ColorRanges[2].Vmax=255
};


//在帧srcFrame中找与正方形停机坪颜色匹配的位置retRect
//如果匹配成功，函数返回true，matchRect为目标图像位置；如果不成功，函数返回false
//输入srcFrame待搜索帧
//输出retRect：如果匹配成功的话，matchRect为目标图像位置
bool FindTargetInFrame(Mat srcFrame, int nHeight, Rect2d &matchRect, int minTargetArea) {
    __android_log_print(ANDROID_LOG_ERROR, "FindTarget_jni", "FindTargetInFrame start.....!@");
    Mat srcFramecopy;
    cv::Mat matInRange;
    cvtColor(srcFrame, srcFrame, COLOR_RGB2BGR);
    cvtColor(srcFrame, srcFramecopy, COLOR_RGB2HSV);
    //inRange(srcFrame,Scalar(26,99,124),Scalar(53,255,255),matInRange);//亮色HSV停机坪图片
    // inRange(srcFramecopy,Scalar(0,30,233), Scalar(58,142,255),matInRange);//暗色HSV停机坪图片
    //inRange(srcFramecopy,Scalar(120,5,240), Scalar(146,16,255),matInRange);// 识别白色图案

//    inRange(srcFramecopy, Scalar(63, 24, 177), Scalar(91, 131, 255), matInRange);// 识别浅绿色图案


    for (int i = 0; i < 3; i++) {

//        __android_log_print(ANDROID_LOG_ERROR, "FindTarget_jni",
//                            "tarmac ： nHeight：%d, tracker_config.ColorRanges[i].HeightMin: %d, tracker_config.ColorRanges[i].Hmin: %d ,"
//                            " tracker_config.ColorRanges[i].Smin: %d, tracker_config.ColorRanges[i].Vmin: %d",
//                            nHeight,
//                            tracker_config.ColorRanges[i].HeightMin,
//                            tracker_config.ColorRanges[i].Hmin,
//                            tracker_config.ColorRanges[i].Smin, tracker_config.ColorRanges[i].Vmin);

        if ((tracker_config.ColorRanges[i].HeightMin <= nHeight) &&
            (tracker_config.ColorRanges[i].HeightMax > nHeight)) {
            inRange(srcFramecopy,
                    Scalar(tracker_config.ColorRanges[i].Hmin, tracker_config.ColorRanges[i].Smin,
                           tracker_config.ColorRanges[i].Vmin),
                    Scalar(tracker_config.ColorRanges[i].Hmax, tracker_config.ColorRanges[i].Smax,
                           tracker_config.ColorRanges[i].Vmax), matInRange);// 识别浅绿色图案
            break;
        }
    }


    // inRange(srcFrame,Scalar(113,206,122), Scalar(206,255,233),matInRange);//暗色BGR停机坪图片
    //inRange(srcFrame,Scalar(0,118,59), Scalar(76,255,124),matInRange);//亮色BGR停机坪图片
    // inRange(srcFrame,Scalar(0,206,17), Scalar(206,255,233),matInRange);//全色BGR停机坪图片

    std::vector<std::vector<cv::Point> > contoursMat;//轮廓
    cv::findContours(matInRange, contoursMat, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    __android_log_print(ANDROID_LOG_ERROR, "FindTarget_jni", "Target : contoursMat.size(): %d", contoursMat.size());

    int maxArea = -1;
    int iArea;
    Rect rect;

    for (int i = 0; i < contoursMat.size(); i++) {

        std::vector<cv::Point> temp = contoursMat.at(i);
        rect = cv::boundingRect(temp);
        //if((temp.size()>=4)&&(rect.width/rect.height<1.5)&&(rect.width/rect.height>0.7)&&(rect.width+rect.height>maxArea))
        if ((temp.size() > 1) && (rect.width * rect.height > maxArea)) {
            iArea = i;
            matchRect = rect;
            maxArea = rect.width * rect.height;
        }
    }
//    std::cout << "max area:" << maxArea << " x:" << rect.x << " y:" << rect.y << " width:"
//              << rect.width << " height:" << rect.height << std::endl;


    if (maxArea >= tracker_config.minTargetArea) {
        //  /720 * 1600
        //  /1088 * 2560
        __android_log_print(ANDROID_LOG_ERROR, "FindTarget_result_jni",
                            "Target : maxArea: %d, x: %lf, y: %lf, width: %lf, height: %lf", maxArea,
                            matchRect.x, matchRect.y, matchRect.width, matchRect.height);
        return true;
    } else
        return false;
}