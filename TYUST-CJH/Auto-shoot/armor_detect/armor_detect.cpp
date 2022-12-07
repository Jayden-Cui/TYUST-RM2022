#include"armor_detect/armor_detect.h"
#include"Info_slover/angle_solver.h"
#include"predict/kala.h"


//
///
/// \brief ArmorProcess::media
/// \param a
/// \param b
/// \param c
/// \return
///
/// //
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///

double ArmorProcess::media(double a,double b,double c)
{

    if ((a-b)*(a-c)<0)
    {
        middle = a;
    }

    if ((b-a)*(b-c)<0)
    {
        middle = b;

    }

    if ((c-a)*(c-b)<0)
    {
        middle = c;
    }
    return middle;
}

/****************************************
* @funcName AdvancedPredictForArmorDetect
* @brief    打包好的预测
* @para     RotatedRect &present_armor,RotatedRect &predict_armor
* @return   无
* @date     2022.3.10
* @author   cjh
*****************************************/
bool ArmorProcess::AdvancedPredictForArmorDetect(RotatedRect &present_armor,RotatedRect &predict_armor)
{
    //如果队列元素不足
    if(armor_queue.size() <= 1)
    {
        armor_queue.push(present_armor);
        armor_queue_time.push(getTickCount());
        return false ;
    }
    else if(armor_queue.size() == 2)
    {
        armor_queue.pop();
        armor_queue.push(present_armor);

        armor_queue_time.pop();                                         //弹出时间
        armor_queue_time.push(getTickCount());                          //压入时间

        bool is_height_min = (armor_queue.back().size.height < armor_queue.back().size.width);
        // 判断装甲板是否发生切换
        bool is_armor_plate_switched = ((fabs(armor_queue.front().angle -armor_queue.back().angle) > 10) ||//角度

                                         fabs((armor_queue.back().size.width / armor_queue.back().size.height) - //H/W
                                         (armor_queue.front().size.width / armor_queue.front().size.height )) > 1 ||

                                         ((fabs(armor_queue.back().center.x - armor_queue.front().center.x) +
                                         fabs(armor_queue.back().center.y - armor_queue.front().center.y))) > 200//中心点
                                         );

        #ifdef USING_DEBUG_ANTISPIN
        if(is_armor_plate_switched)
        {
            last_switched_armor = present_armor;//存储上次切换装甲板
            if(spinning_coeffient == 0 ){//若置信度此时为0
                spinning_coeffient = 2;//设置初值
                predict_armor = present_armor;
                return false;
            }
            else{
                spinning_coeffient =  5 + 6 * spinning_coeffient;
            }

        }
        if(spinning_coeffient < 1)
            spinning_coeffient = 0;
        else if(spinning_coeffient > 4e3)
            spinning_coeffient = 4e3;
        else
            spinning_coeffient /= 2;
        if(spinning_coeffient > 400){
            cout<<"Spinning :"<<spinning_coeffient<<endl;
        }
        else
            cout<<"Steady :"<<spinning_coeffient<<endl;
        #endif//USING_DEBUG_ANTISPIN


        #ifndef USING_DEBUG_ANTISPIN
        if(is_armor_plate_switched)
        {
            predict_armor = present_armor;
            return false;
        }
        #endif//USING_DEBUG_ANTISPIN
        //间隔时间 VX VY  VW VH
        double delta_time = (double)(armor_queue_time.back() - armor_queue_time.front()) / getTickFrequency();//处理时间

         velocity_x = (armor_queue.back().center.x - armor_queue.front().center.x) / delta_time;//Vx
         velocity_y = (armor_queue.back().center.y - armor_queue.front().center.y) / delta_time;  //Vy
         if(velocity_x>800 || velocity_y>800)
         {
             if(abs(velocity_x -lvelocity_x)/lvelocity_x>0.9 || (velocity_x -lvelocity_x)/velocity_x>0.9 )
            velocity_x = media(velocity_x ,lvelocity_x , llvelocity_x);
         if(abs(velocity_y -lvelocity_y)/lvelocity_y>0.9 || (velocity_y -lvelocity_y)/velocity_x>0.9)
             velocity_y = media(velocity_y ,lvelocity_y , llvelocity_y);
         }
         avelocity_x = (velocity_x -lvelocity_x)/ delta_time;//Ax
         avelocity_y = (velocity_y -lvelocity_y)/ delta_time;  //Ay
         if(avelocity_x<0.1)
             avelocity_x=0;
         if(avelocity_y<0.1)
             avelocity_y=0;


         llvelocity_x  = lvelocity_x;
         llvelocity_y  = lvelocity_y;

         lvelocity_x  = velocity_x;
         lvelocity_y  = velocity_y;


        double velocity_height = (MIN(armor_queue.back().size.height, armor_queue.back().size.width) -
                                    MIN(armor_queue.front().size.height, armor_queue.front().size.width)) / delta_time;
        double velocity_width = (MAX(armor_queue.back().size.height, armor_queue.back().size.width) -
        MAX(armor_queue.front().size.height, armor_queue.front().size.width)) / delta_time;

        cout<<"CCC :"<<velocity_x <<endl;
        kalmanfilter.EstimatedTimeofArrival = delta_time/2  + 0.003;//预测时间为处理时间加响应时间//+ 0.05
        cout<<"ETA :"<<kalmanfilter.EstimatedTimeofArrival<<endl;
        kalmanfilter.KF.transitionMatrix = (Mat_<float>(8, 8) << 1,0,0,0,kalmanfilter.EstimatedTimeofArrival,0,0,0,//x
                                                                0,1,0,0,0,kalmanfilter.EstimatedTimeofArrival,0,0,//y
                                                                0,0,1,0,0,0,kalmanfilter.EstimatedTimeofArrival,0,//width
                                                                0,0,0,1,0,0,0,kalmanfilter.EstimatedTimeofArrival,//height
                                                                0,0,0,0,1,0,0,0,//Vx
                                                                0,0,0,0,0,1,0,0,//Vy
                                                                0,0,0,0,0,0,1,0,//Vwidth
                                                                0,0,0,0,0,0,0,1);//Vheight


        //设置测量矩阵
        Mat measure =(Mat_<float>(8, 1) << armor_queue.back().center.x,
                                           armor_queue.back().center.y,
                                           MAX(armor_queue.back().size.height, armor_queue.back().size.width),
                                           MIN(armor_queue.back().size.height, armor_queue.back().size.width),
                                           velocity_x,
                                           velocity_y,
                                           velocity_width,
                                           velocity_height);

        //设置状态转移矩阵

        Mat predict = kalmanfilter.KF.predict();

        if(is_height_min)//设置预测装甲
            predict_armor = RotatedRect(Point2f(predict.at<float>(0,0),predict.at<float>(0,1)),
            Size2f(MAX(armor_queue.back().size.height, armor_queue.back().size.width),MIN(armor_queue.back().size.height, armor_queue.back().size.width)),
            present_armor.angle);
        else //设置预测装甲
            predict_armor = RotatedRect(Point2f(predict.at<float>(0,0),predict.at<float>(0,1)),
            Size2f(MIN(armor_queue.back().size.height, armor_queue.back().size.width),MAX(armor_queue.back().size.height, armor_queue.back().size.width)),
            present_armor.angle);

        kalmanfilter.KF.correct(measure);
        //速度调用为线性预测作准备


    if((fabs(predict_armor.center.x - present_armor.center.x) +
        fabs(predict_armor.center.y - present_armor.center.y)) > 200)//中心点
        {
            predict_armor = present_armor;
            return false;
        }
    }
    return true;

}

////////////////////////////////////////////////////

/****************************************
* @funcName ArmorDetect
* @brief    打包好的自瞄处理全过程
* @para     cv::Mat src_image, ArmorPosture &fight_info, UsbSerial usb_serial, Rect2d roi_rect
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
void ArmorProcess::ArmorDetect(cv::Mat src_image, ArmorPosture &fight_info, UsbSerial serial_usb, Rect2d roi_rect,int dish,double tP,double tY)
{

    cv::RotatedRect target_rect;
    vector<armor>match_armor_list;//匹配成功的装甲
    vector<armor>final_armor_list;
        vector<RotatedRect>final_armor_rect_list;
    AngleSolver angle_slover;
    uint count = 0;

    uint8_t pc_data[10];

    roi_rect = GetRoi(src_image,fight_info);
    Mat roi_image = src_image(roi_rect);
    Point2f offset_roi_point(float(roi_rect.x) ,float(roi_rect.y) );
    Point2f roi_center(float(roi_rect.width/2), float(roi_rect.height/2));
    ArmorDetecter(roi_image,roi_rect,fight_info,match_armor_list);



    /*****
     * ****
     * ***  选  择  目  标  装  甲*/
    if(match_armor_list.size() > 0)
    {

        for(auto armor_rect:match_armor_list)
        {
            final_armor_rect_list.push_back(armor_rect.Rect);
        }

        if(fight_info.last_find_target == true)
        {
            int min_distance = 10000;

            for(auto armor_rect: final_armor_rect_list)
            {
                auto tem_distance_x = abs(abs(static_cast<int>(armor_rect.center.x)) - abs(static_cast<int>(fight_info.last_rect.center.x)));
                if(tem_distance_x <= min_distance)
                {
                    min_distance = tem_distance_x;
                    target_rect = armor_rect;

                }

            }

        }
        else
        {
                float min_distance = 1e8;
                for(auto armor_rect:final_armor_rect_list)
                {
                    auto temp_distance_x = fabs(armor_rect.center.x - roi_center.x);
                    auto temp_distance_y = fabs(armor_rect.center.y - roi_center.y);

                    if(temp_distance_x + temp_distance_y <= min_distance)
                    {
                        min_distance = temp_distance_x + temp_distance_y;
                        target_rect = armor_rect;
                    }

                }

        }


        fight_info.target_rect = target_rect;

        fight_info.last_find_target = true;

        auto armor_ratio = max(target_rect.size.width, target_rect.size.height) / min(target_rect.size.width, target_rect.size.height);

        fight_info.is_small = armor_ratio > 3.3f? false : true;


        fight_info.order = 1;
        GimbalPose ypr;
        ypr.pitch = tP;
        ypr.yaw = tY;



        angle_slover.PNPSolver(target_rect, fight_info, roi_image,ypr);


        //Point2f point_tmp[4];
        Point2f point_2d[4];


         circle(roi_image,target_rect.center,15,Scalar(255,0,0),2,8,0);
         Point2f centerpp;
         centerpp.x = target_rect.center.x + fight_info.roi_offset_x-10;
         centerpp.y = target_rect.center.y + fight_info.roi_offset_y;
         circle(roi_image,centerpp,15,Scalar(0,0,255),2,8,0);
        drawRotatedRect(roi_image, target_rect, Scalar(0, 255, 0), 1);

        cv::Point2f vertices[4];
        fight_info.target_rect.points(vertices);
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
        point_2d[0] = lu;
        point_2d[1] = ru;
        point_2d[2] = ld;
        point_2d[3] = rd;

        vector<Point2f>points_roi_tmp;

        for(int i =0; i <4; i++)

        {
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
        }

        rectangle(src_image, roi_rect, Scalar(0, 255, 0), 1);

        last_target_ = boundingRect(points_roi_tmp);

                //lost_cnt = 0;


       fight_info.target_rect = target_rect;
       fight_info.last_rect = target_rect;
       final_armor_list.clear();
       fight_info.isTrue = true;
       lost_cnt = 0;

       auto center_x_ratio = fight_info.target_rect.center.x - fight_info.last_rect.center.x;

       if(fight_info.speed_buffer.empty() || fight_info.speed_buffer.size() < 6)
       {
           fight_info.speed_buffer.push_back(center_x_ratio);
       }
       else if(fight_info.speed_buffer.size() == 2000)
       {
           fight_info.speed_buffer.pop_front();
           fight_info.speed_buffer.push_back(center_x_ratio);
       }
       }

       else
       {
        fight_info.t_start=0;
        VisionData vdata = {tY,tP,0,0,0,0,0};

        serial_usb.TransformData(vdata);
        serial_usb.send();

           lost_cnt ++;
           if(lost_cnt > 30)//30
           {
               fight_info.order = 0;//0
               cv::RotatedRect rect_temp(cv::Point2f(0,0),cv::Point2f(1280,0),cv::Point2f(1280,1024));
               fight_info.target_rect = rect_temp;
               fight_info.isTrue = false;


              fight_info.last_find_target = false;
              fight_info.speed_buffer.clear();

           }

       }

    if(dish>2280 )
     {
  fight_info.isTrue = false;

         VisionData vdata = {tY,tP,0,0,0,0,0};

         serial_usb.TransformData(vdata);
         serial_usb.send();



     }

     else

    {

    if(fight_info.distance<800)
   {
        VisionData   vdata = {(float)fight_info.armor_yaw,(float)fight_info.armor_pitch,(float)fight_info.distance,1,1,0,1};
        serial_usb.TransformData(vdata);
        serial_usb.send();
    }
    else{
    VisionData   vdata = {(float)fight_info.armor_yaw,(float)fight_info.armor_pitch,(float)fight_info.distance,0,1,0,1};
    serial_usb.TransformData(vdata);
    serial_usb.send();
    }
}
    }

void armor::draw_armor(Mat &img, Point2f roi_offset_point) const
{
    rectangle(img,rect + Point_<int>(roi_offset_point),Scalar(0, 255, 0),1);
}



armor::armor(){}


/****************************************
* @funcName armor
* @brief    构造函数
* @para     const LED_Stick &L1,const LED_Stick &L2
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
armor::armor(const LED_Stick &L1,const LED_Stick &L2)
{
   if(L1.rect.center.x < L2.rect.center.x)
   {
       led_stick[0] = L1;
       led_stick[1] = L2;
   }
   else
   {
       led_stick[0] = L2;
       led_stick[1] = L1;
   }
    error_angle = fabs(L1.rect.angle - L2.rect.angle);

    rect.width = abs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
    rect.height = static_cast<int>((L1.rect.size.height + L2.rect.size.height) /2);
    center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) /2);
    center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) /2);
    rect.x = center.x - rect.width/3;
    rect.y = center.y - rect.height/3;
    rect.width *=2.0 /3;
    rect.height *=2.0/3;

    Rect = drawArmorRect(led_stick[0].rect,led_stick[1].rect);
}


void armor::match_max(std::vector<LED_Stick>&LED,size_t i,size_t j)
{
    cv::RotatedRect R,L;
    if(led_stick[0].rect.center.x > led_stick[1].rect.center.x)
    {
        R = led_stick[0].rect;
        L = led_stick[1].rect;
    }
    else
    {
        R = led_stick[1].rect;
        L = led_stick[0].rect;
    }

    float angle_8 = L.angle - R.angle;
    if(angle_8 < 1e-3f)
        angle_8 = 0.0f;

    float f = error_angle + 0.5*angle_8;

    if(!LED.at(i).matched && !LED.at(j).matched )
        {

            LED.at(i).matched = true;
            LED.at(i).match_index = j;
            LED.at(j).matched = true;
            LED.at(j).match_index = i;
            LED.at(i).match_factor = f;
            LED.at(j).match_factor = f;
        }
        if(LED.at(i).matched && !LED.at(j).matched)
        {
            if(f < LED.at(i).match_factor)
            {
                LED.at(LED.at(i).match_index).matched = false;
                LED.at(i).match_factor = f;
                LED.at(i).match_index = j;
                LED.at(j).matched = true;
                LED.at(j).match_factor = f;
                LED.at(j).match_index = i;

            }
        }
        if(LED.at(j).matched && !LED.at(i).matched)
        {
            if(f < LED.at(j).match_factor )
            {
                LED.at(LED.at(j).match_index).matched = false;
                LED.at(j).match_factor = f;
                LED.at(j).match_index = i;
                LED.at(i).matched = true;
                LED.at(i).match_factor = f;
                LED.at(i).match_index = j;
            }
        }
        if(LED.at(j).matched && LED.at(i).matched
                && LED.at(i).match_factor > f && LED.at(j).match_factor > f)
        {
            LED.at(LED.at(j).match_index).matched = false;
            LED.at(LED.at(i).match_index).matched = false;
            LED.at(i).matched = true;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
            LED.at(j).matched = true;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;

        }


}

/****************************************
* @funcName drawArmorRect
* @brief    得到旋转矩阵
* @para     RotatedRect &left_rect,RotatedRect &right_rect
* @return   RotatedRect(center,cv::Size2f(width,height),(angle*180.0f)/static_cast<float>(CV_PI))
* @date     2021.4.27
* @author   wjl
*****************************************/
RotatedRect armor::drawArmorRect(RotatedRect &left_rect,RotatedRect &right_rect)
{
    const cv::Point&pl=left_rect.center,&pr=right_rect.center;
        cv::Point2f center;
        center.x=(pl.x+pr.x)/2.0f;
        center.y=(pl.y+pr.y)/2.0f;
        cv::Size2f wh_l=left_rect.size;
        cv::Size2f wh_r=right_rect.size;
        //float width = pl.x + pr.x;
        float width=sqrt((pl.x-pr.x)*(pl.x-pr.x)+(pl.y-pr.y)*(pl.y-pr.y));
        float height=std::max(wh_l.height,wh_r.height);
        float angle=std::atan2(right_rect.center.y-left_rect.center.y,right_rect.center.x-left_rect.center.x);
        return cv::RotatedRect(center,cv::Size2f(width,height),(angle*180.0f)/static_cast<float>(CV_PI));
}

bool armor::size_istrue(void) const
{

    if(led_stick[0].rect.size.height * 0.7f < led_stick[1].rect.size.height)
    {
        float armor_width = fabs(led_stick[0].rect.center.x - led_stick[1].rect.center.x);
        if(armor_width > led_stick[0].rect.size.width
                &&armor_width > led_stick[0].rect.size.width
                &&armor_width > (led_stick[0].rect.size.width + led_stick[1].rect.size.width)/3)
        {
            float h_max =(led_stick[0].rect.size.height + led_stick[1].rect.size.height) / 2.0f;

            if(fabs(led_stick[0].rect.size.height + led_stick[1].rect.size.height) < 0.8f * h_max)
            {
                if(h_max *4.0f > rect.width && h_max < 1.2f*rect.width)
                {
                    return true;
                }
            }
        }
    }
    return false;
}


/****************************************
* @funcName GetRoi
* @brief    得到ROI区域
* @para     const cv::Mat &img,ArmorPosture &fight_info
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
cv::Rect ArmorProcess::GetRoi(const cv::Mat &img,ArmorPosture &fight_info)
{
    cv::RotatedRect temp_rect = fight_info.target_rect;
    Size img_size = img.size();
    Rect rect_tmp = last_target_;
    Rect rect_roi;

    if(fight_info.order == 1)
    {

     float scale = 3;

     int w = int(rect_tmp.width * scale);

     if(w < 640)
        w = 640;
     int h = int(rect_tmp.height * scale);
     if(h < 480)
        h = 480;
     int x = int(rect_tmp.x - (w - rect_tmp.width) * 0.5f);
     int y = int(rect_tmp.y - (h - rect_tmp.height) * 0.5f);

     rect_roi = Rect(x,y,w,h);

     fight_info.roi_offset_x = x;
     fight_info.roi_offset_y = y;


     if(makeRectSafe(rect_roi,img_size) == false)
     {
         rect_roi = Rect(0,0,img_size.width,img_size.height);
     }
     return  rect_roi;

}

    else
    {
        rect_tmp = Rect(0,0,img_size.width,img_size.height);
        rect_roi = Rect(0,0,img_size.width,img_size.height);
        return  rect_roi;
    }
}

/****************************************
* @funcName get_average_intensity
* @brief    处理装甲中有灯条的情况
* @para     const Mat &img
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
int armor::get_average_intensity(const Mat &img)
{
    if(rect.width < 1 ||rect.height < 1 || rect.x < 1 || rect.y < 1
       || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return  255;
    Mat roi = img(Range(rect.y,rect.y + rect.height),Range(rect.x,rect.x + rect.width));
    average_intensity = static_cast<int>(mean(roi).val[0]);
    return  average_intensity;
}

/****************************************
* @funcName AdjustRotatedRect
* @brief    调整矩形的角度,纠正长宽
* @para     cv::RotatedRect &rect
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
void ArmorProcess::AdjustRotatedRect(cv::RotatedRect &rect)
{
    if(rect.size.width>rect.size.height)
    {
    const auto temp=rect.size.height;
    rect.size.height=rect.size.width;
    rect.size.width=temp;
    rect.angle+=90;

    }

}

/****************************************
* @funcName drawRotatedRect
* @brief    框出矩形框
* @para     cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness
* @return   tu
* @date     2021.4.27
* @author   wjl
*****************************************/
RotatedRect ArmorProcess::drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f Vertex[4];
         rect.points(Vertex);
         for(int i = 0 ; i < 4 ; i++)
         {
             cv::line(img , Vertex[i] , Vertex[(i + 1) % 4] , color , thickness);
         }

}

//RotatedRect armor::drawArmorRect(LED_Stick L1, LED_Stick L2)
//{
//    const cv::Point&pl=L1.rect.center,&pr=L2.rect.center;
//    cv::Point2f center;
//    center.x=(pl.x+pr.x)/2.0f;
//    center.y=(pl.y+pr.y)/2.0f;
//    cv::Size2f wh_l=led_stick[0].rect.size;
//    cv::Size2f wh_r=led_stick[0].rect.size;
//        //float width = pl.x + pr.x;
//    float width=sqrt((pl.x-pr.x)*(pl.x-pr.x)+(pl.y-pr.y)*(pl.y-pr.y));
//    float height=std::max(wh_l.height,wh_r.height);
//    float angle=std::atan2(led_stick[0].rect.center.y-led_stick[0].rect.center.y,led_stick[0].rect.center.x-led_stick[0].rect.center.x);
//    rect = cv::RotatedRect(center,cv::Size2f(width,height),(angle*180.0f)/static_cast<float>(CV_PI));
//}

/****************************************
* @funcName ArmorDetecter
* @brief    装甲处理具体过程
* @para     cv::Mat &src_image,Rect roi_rect,ArmorPosture &fight_info,vector<armor>&final_armor_list
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
void ArmorProcess::ArmorDetecter(cv::Mat &src_image,Rect roi_rect,ArmorPosture &fight_info,vector<armor>&final_armor_list)
{

    ArmorPlate present_armor;                 //现在的装甲板类
    ArmorPlate predict_armor;                //卡尔曼预测得到的所需击打的装甲板

    Point2f offset_roi_point(roi_rect.x,roi_rect.y);
    vector<LED_Stick>LED_Stick_v;
    cv::Mat gray_img;
    cv::Mat sub_img;
    cv::Mat brightness_img;
    cv::Mat color_img;
    cv::Mat AND_img;
    cv::Mat light_img_black=cv::Mat::zeros(src_image.size(),CV_8UC3);

    vector<cv::Mat>bgr_channels;
    vector<cv::RotatedRect>lights;

    cv::cvtColor(src_image,gray_img,CV_BGR2GRAY);

    cv::split(src_image,bgr_channels);

    cv::Mat element_erode=getStructuringElement(cv::MORPH_RECT,cv::Size(1,1));


//    namedWindow("b",1);
//    namedWindow("c",1);
//    createTrackbar("Threshold","b",&Red_bright_threshold,255,0);
//    createTrackbar("Threshold","c",&Red_color_threshold,255,0);



    if(fight_info.armor_color == BLUE )
{
    threshold(gray_img,brightness_img,Blue_bright_threshold,255,cv::THRESH_BINARY);//亮度二值化  蓝

    subtract(bgr_channels[0],bgr_channels[1],sub_img);//蓝色通道减去绿色通道
    threshold(sub_img,color_img,20,255,cv::THRESH_BINARY);//颜色二值化
    erode(brightness_img, brightness_img, element_erode);

    if(fight_info.distance > 500)
    {
        threshold(gray_img,brightness_img,120,255,cv::THRESH_BINARY);//亮度二值化  蓝
       // bgr_channels[1] = bgr_channels[1]*3;
        subtract(bgr_channels[0],bgr_channels[1],sub_img);//蓝色通道减去绿色通道
        threshold(sub_img,color_img,20,255,cv::THRESH_BINARY);//颜色二值化

    }
}
     if(fight_info.armor_color == RED)//红色仍有问题，待修改
{


         subtract(bgr_channels[2],bgr_channels[1],sub_img);//红色通道减去绿色通道
//         if(fight_info.distance > 500)
//        {
//             threshold(gray_img,brightness_img,70,255,CV_THRESH_BINARY);//亮度二值化  红
//            threshold(sub_img, color_img, 70, 255, CV_THRESH_BINARY);
//        }
//        else
       // {
             threshold(gray_img,brightness_img,Red_bright_threshold,255,cv::THRESH_BINARY);//亮度二值化  红
             threshold(sub_img,color_img,Red_color_threshold,255,cv::THRESH_BINARY);//颜色二值化
             //erode(color_img, color_img, element_erode);

       // }
}
    cv::Mat element=getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    dilate(brightness_img,brightness_img,element);

    AND_img=brightness_img & color_img;
    //rdilate(AND_img, AND_img, element);
   // erode(AND_img, AND_img, element_erode);
//     erode(AND_img, AND_img, element);
//     dilate(AND_img, AND_img, element);
//     dilate(AND_img, AND_img, element);
//     dilate(AND_img, AND_img, element);
    dilate(AND_img, AND_img, element);
    //dilate(AND_img, AND_img, element);
     vector<vector<cv::Point>>AND_contours;
     cv::findContours(AND_img,AND_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

     vector<vector<cv::Point>> brightness_contours;
     cv::findContours(brightness_img,brightness_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

     //lights.reserve(brightness_contours.size());

//     std::vector<int> is_process(brightness_contours.size());

     for(uint i = 0; i < AND_contours.size(); i++)
     {

           double area = contourArea(AND_contours[i]);

           if(area < 20.0||area > 20000.0)continue;//50
           cout<<"SX1"<<area<<endl;
           double lenth = arcLength(AND_contours[i],true);
           if(lenth < 20||lenth > 4000)continue;
           cout<<"SX2"<<lenth<<endl;
           cv::RotatedRect light_bar = minAreaRect(AND_contours[i]);//得到灯条


           AdjustRotatedRect(light_bar);
//           double angle = light_bar.angle;
//           angle = 90 - angle;
//           angle = angle <0.0?angle + 180 : angle;
//           float delta_angle = abs(angle-90);
//           if(delta_angle>35)continue;//15
//           cout<<"SX3"<<delta_angle<<endl;
           auto rect = std::minmax(light_bar.size.width,light_bar.size.height);
           double light_aspect_ratio = max(rect.second, rect.first) / min(rect.first, rect.second);
           auto angle = light_bar.angle;
           cout<<"SX3"<<light_aspect_ratio<<endl;

           if((light_aspect_ratio>2.1)&&(light_aspect_ratio<13))


            if ( light_bar.size.area() < 20000 && abs(angle) < 45.0f)
           {
                cout<<"SX4"<<abs(angle)<<endl;
                LED_Stick r(light_bar);
                LED_Stick_v.emplace_back(r);
           }
         }


     //imshow("sub_img",sub_img);
     //imshow("灰度图",gray_img);
//     imshow("b",brightness_img);
//     imshow("c",color_img);
//     imshow("与运算图",AND_img);


/*****
 * *  寻找可能的装甲板
*****/
     for(size_t i = 0;i < LED_Stick_v.size(); i++)
     {

         for(size_t j = i + 1; j <LED_Stick_v.size(); j++)
         {

             armor Armor(LED_Stick_v.at(i),LED_Stick_v.at(j));
             if(Armor.error_angle < 9.0f)
             {

                 cv::RotatedRect &l_rect = LED_Stick_v[i].rect;
                 cv::RotatedRect &r_rect = LED_Stick_v[j].rect;
                 float delata_height = GET_DIST(l_rect .size.height,r_rect .size.height);
                 if(delata_height>100)continue;
                 cout<<"SX5"<<delata_height<<endl;
                 float area_ratio;
                 if(r_rect.size.area() >= l_rect.size.area())
                 {
                     area_ratio = l_rect.size.area() * 1.0f / r_rect.size.area();
                 }
                 else
                 {
                     area_ratio = r_rect.size.area() * 1.0f / l_rect.size.area();
                 }
                 if(area_ratio <= 0.25f)continue;

                 //if(h_max *4.0f > rect.width && h_max < 1.2f*rect.width)continue;

//                 float angle_diff = abs(l_rect.angle - r_rect.angle);
//                 if(angle_diff > 9)continue;

                 float delta_x = abs(r_rect.center.x - l_rect.center.x);
                 float delta_y = abs(r_rect.center.y - l_rect.center.y);


                float delta_xratio = delta_x * 1.0 / r_rect.size.height;
                 if(delta_xratio<1 ||delta_xratio>9)continue;


                 //                 float sub_dis_max;

                 //                 if(delta_x >= delta_y)
                 //                 {
                 //                     sub_dis_max = delta_x;
                 //                 }
                 //                 else
                 //                 {
                 //                     sub_dis_max = delta_y;
                 //                 }

                 //                // if(sub_dis_max > 1280)continue;
                 //                 if(delta_x < 20)continue;
                 //                 //cout<<"delta_x:"<<delta_x<<endl;
                 //                // cout<<"delta_y:"<<delta_y<<endl;

                 //                 //if(delta_x < 40)continue;

                 //                 //if(delta_y > 80)continue;

                 float deviationAngle = abs(atan(delta_y  / delta_x )) * 180 / CV_PI;
                 if(deviationAngle > 50)continue;
                 cout<<"deviationAngle"<<deviationAngle<<endl;

                 float meanLen = (l_rect.size.height + r_rect.size.height) / 2;
                 float yDiff = abs(l_rect.center.y - r_rect.center.y);
                 float yDiff_ratio = yDiff / meanLen;
                 if(yDiff_ratio > 0.9f)continue;

                 float xDiff = abs(l_rect.center.x - r_rect.center.x);
                 float xDiff_ratio = xDiff / meanLen;
                 if(xDiff_ratio > 4.7f)continue;

                 float length_diff = abs(l_rect.size.height - r_rect.size.height);
                 float lengthDiffRation = length_diff / max(l_rect.size.height,r_rect.size.height);
                 if(lengthDiffRation > 0.5f)continue;

                 if(Armor.get_average_intensity(gray_img) > 160)continue;//排除两灯条中间有灯条的情况
                 Armor.match_max(LED_Stick_v,i,j);


             }
         }
     }


     /*****
      * *  根据匹配状态得到正确装甲板
     *****/
     for(size_t i = 0; i < LED_Stick_v.size();i++)
     {
         if(LED_Stick_v.at(i).matched)
         {


             LED_Stick_v.at(LED_Stick_v.at(i).match_index).matched = false;
             armor Armor(LED_Stick_v.at(i),LED_Stick_v.at(LED_Stick_v.at(i).match_index));






             float rect_h = min(Armor.Rect.size.height, Armor.Rect.size.width);
             float rect_w = max(Armor.Rect.size.height, Armor.Rect.size.width);
             float ratio_h_w = rect_w * 1.0f / rect_h;

//             cout<<"ratio_h_w:"<<ratio_h_w<<endl;

//             cout<<"Armor.area:"<<Armor.Rect.size.area()<<endl;

//             cout<<"Armor.height:"<<Armor.Rect.size.height<<endl;

//             cout<<"Armor.angle:"<<Armor.Rect.angle<<endl;

//             if(Armor.Rect.size.height > 100)
//                 continue;

//             if(Armor.Rect.size.area() < 300)
//                 continue;

//             if(Armor.Rect.size.area() > 18000)
//                 continue;

             if(ratio_h_w > 5.2f)
             continue;

             if(ratio_h_w < 1.3f)
                 continue;

//             if(abs(Armor.Rect.angle) > 25)
//                continue;

           //  if(Armor.Rect.size.height * 2 / 4 > Armor.Rect.size.width)continue;



            //滤波
//             present_armor.boundingRect = Armor.Rect;
//             AdvancedPredictForArmorDetect(present_armor.boundingRect,predict_armor.boundingRect);//对装甲板进行卡尔曼预测与反陀螺(实验性)

//             if(abs(predict_armor.boundingRect.center.x - present_armor.boundingRect.center.x) >30 && abs(predict_armor.boundingRect.center.x - present_armor.boundingRect.center.x) <500)
//           {
//                 cout<<"???????????????????????????????//"<<predict_armor.boundingRect.center << present_armor.boundingRect.center<<endl;
//                 Point2f predict_vector = predict_armor.boundingRect.center - present_armor.boundingRect.center;
//                         for(int i = 0; i < 4 ;i++)
//                             predict_armor.apex[i] = present_armor.apex[i] + predict_vector;

//             Armor.Rect = predict_armor.boundingRect;

//             }

             final_armor_list.push_back(Armor);
         }
     }



}
