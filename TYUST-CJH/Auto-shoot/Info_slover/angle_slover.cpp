#include"angle_solver.h"
#include"main/visual_proc.h"
#include"armor_detect/armor_detect.h"
#include"predict/kala.h"
#include"Info_slover/angle_solver.h"
#define PI 3.141592





AngleSolver::AngleSolver(): m_x_off{0.f}, m_y_off{0.f}, m_z_off{0.f}
{
        setOffParam();
    Eigen::MatrixXd inputP(6, 6);		//初始化状态协方差矩阵
    inputP <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 1.0;

    Eigen::MatrixXd inputQ(6, 6);		//初始化过程噪声矩阵
    inputQ <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 50.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 50.0;
//    inputQ <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
//                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
//                0.0, 0.0, 0.0, 50.0, 0.0, 0.0,
//                0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
//                0.0, 0.0, 0.0, 0.0, 0.0, 50.0;

    Eigen::MatrixXd inputH(3, 6);		//初始化观测矩阵
    inputH <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    Eigen::MatrixXd inputR(3, 3);		//初始化观测噪声矩阵
    inputR <<   100.0, 0.0, 0.0,
                0.0, 100.0, 0.0,
                0.0, 0.0, 100.0;


    _coordinateFilter.InitParam(inputP, inputQ, inputH, inputR);

    _coordinateFilter.setLimit(cv::Point3f(MAX_COORDINATE_XZ, MAX_COORDINATE_Y, MAX_COORDINATE_XZ), cv::Point3f(FILTER_COORDINATE_MAX, FILTER_COORDINATE_MAX, FILTER_COORDINATE_MAX));

}

//xnz ynx zny

cv::Point3f AngleSolver::changeCoor(cv::Point3f point)
{
    cv::Mat trans_mat = GetTransMat(m_x_off, m_y_off, m_z_off);
    cv::Mat target    = (cv::Mat_<float>(4, 1) << point.x, point.y, point.z, 1);
    cv::Mat_<float> target_in_base = trans_mat * target;
    return cv::Point3f(target_in_base.at<float>(0), target_in_base.at<float>(1), target_in_base.at<float>(2));
}
/**
     *@brief: 根据装甲板坐标、当前云台位姿、射速等信息计算目标位姿
     */
    GimbalPose AngleSolver::getAngle(cv::Point3f position, GimbalPose cur_pose, double bullet_speed)
    {

        m_bullet_speed = bullet_speed;
        GimbalPose solve_angle;
        cv::Point3f gun_base   = changeCoor(position);//加平移矩阵
        double pitch_in_camera = std::atan(gun_base.z / std::sqrt(gun_base.y * gun_base.y + gun_base.x * gun_base.x))*44;
        double yaw_in_camera   = std::atan(gun_base.y / gun_base.x)*60;

        solve_angle.pitch =  cur_pose.pitch - pitch_in_camera + m_pitch_off;//cur_pose.pitch +
        solve_angle.yaw   =  cur_pose.yaw   - yaw_in_camera   + m_yaw_off;//cur_pose.yaw   +
        if(m_fix_on)
        {
            double dis = sqrt(pow(gun_base.x, 2) + pow(gun_base.z, 2));
            double angle_fix = 0.5* (asin((9.8 * dis * pow(cos(solve_angle.pitch), 2)) / pow(m_bullet_speed, 2) - sin(solve_angle.pitch)) + solve_angle.pitch);
            solve_angle.pitch -= angle_fix;
            cout<<"!!!!!!!!!!!!!!!!!!!!!!"<<angle_fix<<endl;
        }
        return solve_angle;
    }

    /**
     *@brief: 将装甲板坐标从相对坐标转化为绝对坐标
     */
    cv::Point3f AngleSolver::cam2abs(cv::Point3f position, GimbalPose cur_pose)
    {
        cv::Mat rot_mat    = GetRotMatXYZ(0.0, cur_pose.pitch, cur_pose.yaw);
        cv::Mat trans_mat  = GetTransMat(m_x_off, m_y_off, m_z_off);
        cv::Mat tf_mat     = rot_mat * trans_mat;
        cv::Mat target     = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
        cv::Mat abs_target = tf_mat * target;

        return cv::Point3f(abs_target.at<float>(0), abs_target.at<float>(1), abs_target.at<float>(2)); // X Y Z
    }

    /**
     *@brief: 将装甲板坐标从绝对坐标转化为相对坐标
     */
    cv::Point3f AngleSolver::abs2cam(cv::Point3f position, GimbalPose cur_pose)
    {
        cv::Mat rot_mat    = GetRotMatXYZ(0.0, cur_pose.pitch, cur_pose.yaw);
        cv::Mat trans_mat  = GetTransMat(m_x_off, m_y_off, m_z_off);
        cv::Mat tf_mat     = rot_mat * trans_mat;
        cv::Mat target     = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
        cv::Mat gun_target = tf_mat.inv() * target;

        return cv::Point3f(gun_target.at<float>(0), gun_target.at<float>(1), gun_target.at<float>(2)); // X Y Z
    }

    cv::Point3f AngleSolver::cam2gun(cv::Point3f position)
    {
        cv::Mat trans_mat  = GetTransMat(m_x_off, m_y_off, m_z_off);
        cv::Mat target     = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
        cv::Mat gun_target = trans_mat * target;

        return cv::Point3f(gun_target.at<float>(0), gun_target.at<float>(1), gun_target.at<float>(2));
    }

    void AngleSolver::setOffParam()
    {
      //cv::FileStorage fs(ANGLE_CFG, cv::FileStorage::READ);
        m_x_off = 0;
        m_y_off = 0;
        m_z_off = 0;
        m_pitch_off = 0;
        m_yaw_off = 0;
        m_chx_off = 0;
        m_chy_off = 0;
        m_chz_off = 0;
        m_fix_on = 0;

    }
//X轴的旋转矩阵
    cv::Mat GetRotMatX(float x)
    {
        return cv::Mat_<float>(4, 4) <<
               1,  0,      0,          0,
               0,  cos(x), -sin(x),    0,
               0,  sin(x), cos(x),     0,
               0,  0,      0,          1;
    }
//y轴的旋转矩阵
    cv::Mat GetRotMatY(float y)
    {
        return cv::Mat_<float>(4, 4) <<
               cos(y),     0,  sin(y), 0,
               0,          1,  0,      0,
               -sin(y),    0,  cos(y), 0,
               0,          0,  0,      1;
    }
//z轴的旋转矩阵
    cv::Mat GetRotMatZ(float z)
    {
        return cv::Mat_<float>(4, 4) <<
               cos(z), -sin(z),    0,  0,
               sin(z), cos(z),     0,  0,
               0,      0,          1,  0,
               0,      0,          0,  1;
    }
//相机坐标矩阵
    cv::Mat GetTransMat(float x, float y, float z)
    {
        return cv::Mat_<float>(4, 4) <<
               1,  0,  0,  x,
               0,  1,  0,  y,
               0,  0,  1,  z,
               0,  0,  0,  1;
    }

    cv::Mat GetRotMatXYZ(float x, float y, float z)
    {
        return GetRotMatZ(z) * GetRotMatY(y) * GetRotMatX(x);
    }


/****************************************
* @funcName PNPSolver
* @brief    角度结算
* @para     无
* @return   无
* @date     2022.3.10
* @author   cjh
*****************************************/
void AngleSolver::PNPSolver(cv::RotatedRect object_rect,ArmorPosture &fight_info, cv::Mat &frame,GimbalPose &ypr)
{
          //static   MotionPredict _cprediction;
    Mat DISTANCE = Mat::zeros(frame.size() , CV_8UC3);

    Mat rot_vector,translation_vector;
    vector<Point2f> object2d_point;


    getTarget2dPoints(object_rect, object2d_point, fight_info);
    if(fight_info.is_small == true)
    {
    Get_small_AngleDistance(object2d_point, rot_vector, translation_vector);
    cout<<"small"<<endl;
    }
    else
    {
    Get_big_AngleDistance(object2d_point, rot_vector, translation_vector);
    cout<<"big"<<endl;
    }


    _worldOriginInCamera.x = translation_vector.at<double>(0,0)*10;
    _worldOriginInCamera.y = translation_vector.at<double>(1,0) *10;
    _worldOriginInCamera.z = translation_vector.at<double>(2,0)*10;

  //  double ndis  = sqrt(_worldOriginInCamera.x+ _worldOriginInCamera.y+ _worldOriginInCamera.z);
//    cout<<"SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS"<<_worldOriginInCamera.x<<_worldOriginInCamera.y<<_worldOriginInCamera.z<<endl;


    cv::Point3f CKcoordinate = cv::Point3f(MM_TO_M_TRANS(_worldOriginInCamera.z) , MM_TO_M_TRANS(_worldOriginInCamera.x), MM_TO_M_TRANS(_worldOriginInCamera.y));
    cv::Point3f nDataV;
    CKcoordinate = cam2abs(CKcoordinate, ypr);


//X向前Y左右Z向上





    CKcoordinate = abs2cam(CKcoordinate, ypr);

    fight_info.roi_offset_x = CKcoordinate.x;
    fight_info.roi_offset_y = CKcoordinate.y;

    _coordinateStruct.actualCoordinate = CKcoordinate;
    _coordinateStruct.coordinateTime = _coordinateFilter.getKFtime();
    int  shootSpeed = 24;
    cv::Point3f nKcoordinate = cv::Point3f(M_TO_CM_TRANS(CKcoordinate.x) , M_TO_CM_TRANS(CKcoordinate.y), M_TO_CM_TRANS(CKcoordinate.z));

    GimbalPose finpose;

    finpose = getAngle( CKcoordinate, ypr,  shootSpeed);





    double ntx = M_TO_CM_TRANS(_coordinateStruct.actualCoordinate.x);
    double nty = M_TO_CM_TRANS(_coordinateStruct.actualCoordinate.y);
    double ntz = M_TO_CM_TRANS(_coordinateStruct.actualCoordinate.z);

     cout<<ntx<<"CM"<<nty<<"CM"<<ntz<<"CM"<<endl;

    double ndis  = sqrt(ntx*ntx+ nty*nty+ ntz*ntz);










    //if(tP)

    //cout<<"KKKKKKKKKKKK"<<ntx<<endl;
//    if(abs(nDataV.y)>2e-7)
//    nty += nDataV.y*13*100000;
//    if(abs(nDataV.z)>2e-7)
//    ntz += nDataV.z*13*100000;






    //接下来弹丸速度 水平距离 陀螺仪角度
    //nndata = _prediction.prediction(nndata, _coordinateStruct.coordinateTime, shootSpeed);
    //putText(DISTANCE , to_string(tx) , Point(object_rect.center.x-100, object_rect.center.y-100), cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
    //putText(DISTANCE , to_string(ty) , Point(object_rect.center.x    , object_rect.center.y    ), cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
    //putText(DISTANCE , to_string(tz) , Point(object_rect.center.x+100, object_rect.center.y+100), cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
    //imshow("DISTANCE", DISTANCE);


//        double angle_yaw = atan2(abs(ntx),ntz) * 180 / 3.1415926;
//        double angle_pitch = atan((abs(nty))/ntz * 1.0) * 180 / 3.1515926;





        fight_info.distance = ndis;

       // finpose.yaw=finpose.yaw;
       // finpose.pitch=finpose.pitch;
        fight_info.armor_yaw = finpose.yaw;
        fight_info.armor_pitch = finpose.pitch;



    cout<<"距离值= "<<fight_info.distance<<"cm"<<endl
        <<"yawU=  "<<fight_info.armor_yaw<<endl
        <<"PitchU="<<fight_info.armor_pitch<<endl<<endl;



}

/****************************************
* @funcName Get_small_AngleDistance
* @brief    获得PNP结算必需的小装甲的数据
* @para     const std::vector<cv::Point2f> & points2d, cv::Mat &rot, cv::Mat &trans
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
void AngleSolver::Get_small_AngleDistance(const std::vector<cv::Point2f> & points2d, cv::Mat &rot, cv::Mat &trans)
{
    std::vector<cv::Point3f> point3d;
    //小装甲
    //float half_x = 13.8f/2.0f;  //width_target / 2.0;
    float half_x = 13.8f/2.0f;
    float half_y = 5.6f/2.0f;  //height_target / 2.0;
    //三维点
    point3d.push_back(Point3f(-half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, half_y, 0));
    point3d.push_back(Point3f(-half_x, half_y, 0));

    cv::Mat r;
    Mat cam_matrix = (Mat_<double>(3,3)<<2034.4, -6.2, 663.4457, 0, 2058.11, 672.665, 0, 0, 1);//1981.3, -6.2, 684, 0, 2006.7, 504, 0, 0, 1
    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.08441, 0.7558, -0.00717, 0.0049,0);//-0.1029, 0.0058, -0.0030, 0.0047,0

//    Mat cam_matrix = (Mat_<double>(3,3)<<2173.13068784915, 0, 680.841983916520, 0, 2174.36873724620, 433.428625731238, 0, 0, 1);//1981.3, -6.2, 684, 0, 2006.7, 504, 0, 0, 1
//    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.1232, 0.8539, -0.0093, -0.0025,0);//-0.1029, 0.0058, -0.0030, 0.0047,0

//    Mat cam_matrix = (Mat_<double>(3,3)<<622.10421,0,355.07792,0,629.61051,256.2444,0,0,1);
//    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.42574,0.29455,-0.00056,0.00119,0);
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);
    Rodrigues(r, rot);
}

/****************************************
* @funcName Get_big_AngleDistance
* @brief     获得PNP结算必需的大装甲的数据
* @para     const std::vector<cv::Point2f> & points2d, cv::Mat &rot, cv::Mat &trans
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
void AngleSolver::Get_big_AngleDistance(const std::vector<cv::Point2f> & points2d, cv::Mat &rot, cv::Mat &trans)
{
    std::vector<cv::Point3f> point3d;
    //小装甲
    //float half_x = 13.8f/2.0f;  //width_target / 2.0;
    float half_x = 21.8f/2.0f;
    float half_y = 5.4f/2.0f;  //height_target / 2.0;
    //三维点
    point3d.push_back(Point3f(-half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, half_y, 0));
    point3d.push_back(Point3f(-half_x, half_y, 0));

    cv::Mat r;
    Mat cam_matrix = (Mat_<double>(3,3)<<1981.3, -6.2, 684, 0, 2006.7, 504, 0, 0, 1);//1981.3, -6.2, 684, 0, 2006.7, 504, 0, 0, 1
    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.1029, 0.0058, -0.0030, 0.0047,0);//-0.1029, 0.0058, -0.0030, 0.0047,0
//    Mat cam_matrix = (Mat_<double>(3,3)<<622.10421,0,355.07792,0,629.61051,256.2444,0,0,1);
//    Mat distortion_coeff = (Mat_<double>(5,1)<<-0.42574,0.29455,-0.00056,0.00119,0);
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);
    Rodrigues(r, rot);
}

/****************************************
* @funcName getTarget2dPoints
* @brief    得到装甲的四个顶点
* @para     cv::RotatedRect object_rect,std::vector<Point2f> &object2d_point,ArmorPosture fight_info
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
void AngleSolver::getTarget2dPoints(cv::RotatedRect object_rect,std::vector<Point2f> &object2d_point,ArmorPosture fight_info)
{
    cv::Point2f vertices[4];
    object_rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y) {
        lu = vertices[0];
        ld = vertices[1];
    } else {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y) {
        ru = vertices[2];
        rd = vertices[3];
    } else {
        ru = vertices[3];
        rd = vertices[2];
    }
    lu.x += fight_info.roi_offset_x;
    ru.x += fight_info.roi_offset_x;
    ld.x += fight_info.roi_offset_x;
    rd.x += fight_info.roi_offset_x;

    lu.y += fight_info.roi_offset_y;
    ru.y += fight_info.roi_offset_y;
    ld.y += fight_info.roi_offset_y;
    rd.y += fight_info.roi_offset_y;

    object2d_point.clear();
    object2d_point.push_back(lu);
    object2d_point.push_back(ru);
    object2d_point.push_back(rd);
    object2d_point.push_back(ld);
}



