#ifndef ANGLE_SOLVER_H
#define ANGLE_SOLVER_H

#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include"armor_detect/armor_detect.h"
#include"predict/kala.h"
using namespace std;
using namespace cv;


        cv::Mat GetRotMatX(float x);
        cv::Mat GetRotMatY(float y);
        cv::Mat GetRotMatZ(float z);
        cv::Mat GetRotMatXYZ(float x, float y, float z);
        cv::Mat GetTransMat(float x, float y, float z);



struct GimbalPose
{
    float  pitch;
    float  yaw;
    float  roll;
    double timestamp;

    GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0)
    {
        this->pitch     = pitch;
        this->yaw       = yaw;
        this->roll      = roll;
    }

    GimbalPose operator=(const GimbalPose& gm)
    {
        this->pitch     = gm.pitch;
        this->yaw       = gm.yaw;
        this->roll      = gm.roll;
        this->timestamp = gm.timestamp;
        return *this;
    }

    GimbalPose operator=(const float init_value)
    {
        this->pitch     = init_value;
        this->yaw       = init_value;
        this->roll      = init_value;
        //this->timestamp = wmj::now();
        return *this;
    }

    friend GimbalPose operator-(const GimbalPose& gm1, const GimbalPose gm2)
    {
        GimbalPose temp{};
        temp.pitch     = gm1.pitch - gm2.pitch;
        temp.yaw       = gm1.yaw   - gm2.yaw;
        temp.roll      = gm1.roll  - gm2.roll;
        //temp.timestamp = wmj::now();
        return temp;
    }

    friend GimbalPose operator+(const GimbalPose& gm1, const GimbalPose gm2)
    {
        GimbalPose temp{};
        temp.pitch     = gm1.pitch + gm2.pitch;
        temp.yaw       = gm1.yaw   + gm2.yaw;
        temp.roll      = gm1.roll  + gm2.roll;
        //temp.timestamp = wmj::now();
        return temp;
    }

    friend GimbalPose operator*(const GimbalPose& gm, const float k)
    {
        GimbalPose temp{};
        temp.pitch     = gm.pitch * k;
        temp.yaw       = gm.yaw   * k;
        temp.roll      = gm.roll  * k;
        //temp.timestamp = wmj::now();
        return temp;
    }

    friend GimbalPose operator*(const float k, const GimbalPose& gm)
    {

        GimbalPose temp{};
        temp.pitch     = gm.pitch * k;
        temp.yaw       = gm.yaw   * k;
        temp.roll      = gm.roll  * k;
        //temp.timestamp = wmj::now();
        return temp ;
    }

    friend GimbalPose operator/(const GimbalPose& gm, const float k)
    {
        GimbalPose temp{};
        temp.pitch     = gm.pitch / k;
        temp.yaw       = gm.yaw   / k;
        temp.roll      = gm.roll  / k;
        //temp.timestamp = wmj::now();
        return temp;
    }

    friend std::ostream& operator<<(std::ostream& out, const GimbalPose& gm)
    {
        out << "[pitch : " << gm.pitch << ", yaw : " << gm.yaw << "]";
        return out;
    }
};

class AngleSolver
{


public:

    struct CoordinateStruct {
        cv::Point3f worldOriginInPZT = cv::Point3f(0.0f, 0.0f, 0.0f);   //相机坐标系下的坐标
        cv::Point3f actualCoordinate = cv::Point3f(0.0f, 0.0f, 0.0f);   //固有坐标系下的坐标
        float coordinateTime;
    };
    void PNPSolver(cv::RotatedRect object_rect,ArmorPosture &fight_info, cv::Mat &frame,GimbalPose &ypr);
    void SolveDataProcess(uint8_t* armor_data,ArmorPosture fight_info);
    void SolveguardData(uint8_t* armor_data,ArmorPosture fight_info);

    //void reculate_v(ArmorPosture&fight_info);
    AngleSolver();
    ~AngleSolver(){}
public:
    int short_offset_x_ = 100;
    int short_offset_y_ = 100;
    int long_offset_x_ = 100;
    int long_offset_y_ = 100;
    int preswitch;
    //bool _topFlag = false;


public:
    float m_bullet_speed;

    void setOffParam();
    cv::Point3f cam2abs(cv::Point3f, GimbalPose);
    cv::Point3f abs2cam(cv::Point3f, GimbalPose);
    cv::Point3f cam2gun(cv::Point3f);
    GimbalPose getAngle(cv::Point3f, GimbalPose, double);
   // GimbalPose getAngle(cv::Point3f, GimbalPose);
private:
    cv::Point3f changeCoor(cv::Point3f point);

    float m_x_off;
    float m_y_off;
    float m_z_off;
    float m_yaw_off;
    float m_pitch_off;
    float m_chx_off;
    float m_chy_off;
    float m_chz_off;
    int   m_fix_on;


private:
    void getTarget2dPoints(cv::RotatedRect object_rect,std::vector<Point2f> &object2d_point,ArmorPosture fight_info);
    void Get_small_AngleDistance(const std::vector<cv::Point2f> & points2d, cv::Mat &rot, cv::Mat &trans);
    void Get_big_AngleDistance(const std::vector<cv::Point2f> & points2d, cv::Mat &rot, cv::Mat &trans);
    //void predict(ArmorPosture &fight_info);
    KalmanFilterC _coordinateFilter;
    Prediction _prediction;
    CoordinateStruct _coordinateStruct;

    cv::Point3f _worldOriginInPZTActual; //固有坐标系下的坐标
    cv::Point3f _lastWorldOriginInPZTActual;
    cv::Point3f _worldOriginInPZT;      //世界坐标在平行相机坐标系位置(云台中心为原点)
    cv::Point3f _lastWorldOriginInPZT;
    cv::Point3f _worldOriginInBarrel;   //世界坐标在平行相机坐标系位置(枪管出口为原点)
    cv::Point3f _worldOriginInCamera;   //世界坐标在相机坐标系位置(相机为原点)


    bool _lastTopFlag = false;

    //top
    vector<float> _xHistory;
    vector<float> _yHistory;
    vector<float> _zHistory;
    Point3f dataCC;

    float _maxCoordinate = 0.0;
    float _minCoordinate = 0.0;
};
#endif // ANGLE_SOLVER_H
