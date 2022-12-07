#ifndef VISUAL_PROC_H
#define VISUAL_PROC_H
#include<iostream>
#include<opencv2/opencv.hpp>
#include<mutex>
#include<condition_variable>
#include<cam_driver/open_camera.h>
#include<cam_driver/DVPCamera.h>
#include"predict/predict.h"


enum EnemyType
{
    NO_GET = 0,
    BLUE,
    RED,
    TUOLUO,
    AUTO_SHOOT

};



class VisualProc
{
public:
    VisualProc()
    {
       ArmorType = NO_GET;
       mode = 0;
    }
    uint8_t pc_data[10];
    int U = 1;
    void produce();
    void consumer();
    void WriteVideo();
    void Serial();
    EnemyType ArmorType;
    uint enemy_color = 0;
    float mode;
    float tY;
    float tP;
    float tpv;
    float tyv;

private:
    std::mutex mtx;
    std::vector<cv::Mat> vec_buff;
    std::condition_variable cv_thread;
};
#endif // VISUAL_PROC_H
