#ifndef ARMOR_DETECT_H
#define ARMOR_DETECT_H


#include<opencv2/opencv.hpp>
#include<iostream>
#include"usb_serial/serial_usb.h"
#include"main/visual_proc.h"
#include"predict/predict.h"
#include"shoot_top/shoot_top.h"
#include<opencv2/imgproc/types_c.h>
using namespace cv;
using namespace std;
#define GET_DIST(a,b) std::abs(a-b)/(a>b?a:b)
#define Blue_bright_threshold 175
//#define Red_bright_threshold 100
//int Blue_bright_threshold =130;
//int Red_bright_threshold =100;
class ArmorPlate
{
public:
    cv::RotatedRect boundingRect;
    cv::Point2f apex[4];
    cv::Point2f center;
};

//   灯   条   类
class LED_Stick
{
public:
    LED_Stick():matched(false){}

    LED_Stick(const cv::RotatedRect &R)
    {
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    cv::RotatedRect rect;
    bool matched;
    size_t match_index;
    float match_factor;
};



//  装  甲  板  姿  态 结  算
class ArmorPosture
{
public:
    ArmorPosture(cv::RotatedRect rect)
    {
        armor_yaw = 0.0;
        armor_pitch = 0.0;
        distance = 0.0;
        order = 0;//0
        bullet_speed = 0.0;
        target_rect = rect;
        roi_offset_x = 0;
        roi_offset_y = 0;
        isTrue = false;


    }
    double Adish;
    double armor_yaw;
    double armor_pitch;
    double distance;
    uint8_t order;
    double bullet_speed;
    uint armor_type;
    float roi_offset_x;
    float roi_offset_y;
    int armor_color;
    cv::RotatedRect target_rect;
    cv::RotatedRect last_rect;

    bool last_find_target;
    bool isTrue;

    bool is_small;
    double t_start ;
    int feedback;
    int ismiddle;
    float tyv;
    





    deque<float>speed_buffer;

    cv::Rect ROI;


};



//   装   甲   板  构   成
class armor
{
public:
    armor();
    armor(const LED_Stick &L1, const LED_Stick &L2);

    LED_Stick led_stick[2];
    float error_angle;
    cv::Point2i center;
    //cv::Rect2i  rect;
    int average_intensity;
    Rect2i rect;
    RotatedRect Rect;

    void match_max(std::vector<LED_Stick>&LED,size_t i,size_t j);
    bool size_istrue(void)const;
    RotatedRect drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);
    void draw_armor(Mat &img, Point2f roi_offset_point)const;
    RotatedRect drawArmorRect(RotatedRect &L1,RotatedRect &L2);

    int get_average_intensity(const Mat&img);


};




//  装   甲   板   检   测    器
class   ArmorProcess
{

public:
    void ArmorDetect(cv::Mat src_image, ArmorPosture &fight_info,UsbSerial serial_usb,Rect2d ROI,int dish,double tP,double tY);
    cv::Rect GetRoi(const cv::Mat &img,ArmorPosture &fight_info);
    void ArmorDetecter(cv::Mat &src_image,Rect roi_rect,ArmorPosture &figh_info,vector<armor>&final_armor_list);
    bool AdvancedPredictForArmorDetect(RotatedRect &present_armor,RotatedRect &predict_armor);//装甲板卡尔曼
 double middle;
 float Xxv;
 float Yyv;
 double velocity_x;
 double velocity_y;
 double lvelocity_x;
 double lvelocity_y;
 double llvelocity_x;
 double llvelocity_y;
 double avelocity_x;
 double avelocity_y;
private:


 int Red_bright_threshold =100;
 int Red_color_threshold =60;

    int _sentrymode;
    int _basemode;

    bool is_target_spinning;                    //目标是否进入陀螺模式
    float spinning_coeffient;                      //旋转置信度

    RotatedRect last_switched_armor;            //最后一次切换的装甲板

    queue<RotatedRect> armor_queue;                     //记录辅瞄装甲板集合
    queue<clock_t> armor_queue_time;             //记录上面集合的保存时间

    Kalman kalmanfilter{KALMAN_TYPE_ARMOR};             //设置卡尔曼滤波器类型
private:
    // R O I 参 数
    Rect last_target_;
    int lost_cnt = 0;
    int detect_cnt_ = 0;

    //cv::Rect GetRoi(const cv::Mat &img,ArmorPosture &fight_info);
    bool makeRectSafe(cv::Rect & rect,cv::Size size)
    {
        if(rect.x < 0)
           rect.x = 0;
        if(rect.x + rect.width > size.width)
           rect.width = size.width - rect.x;
        if(rect.y < 0)
           rect.y = 0;
        if(rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if(rect.width <= 0 || rect.height <= 0)
            return  false;
        return  true;

    }

    //void ArmorDetecter(cv::Mat &img,cv::Rect roi_rect,ArmorPosture &figh_info,vector<armor>&final_armor_list);
    void AdjustRotatedRect(cv::RotatedRect &rect);
    RotatedRect drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);
    RotatedRect drawArmorRect(RotatedRect &L1,RotatedRect &L2);
    double media(double a,double b,double c);
    vector<Point2f>points_2d_;
};
#endif // ARMOR_DETECT_H
