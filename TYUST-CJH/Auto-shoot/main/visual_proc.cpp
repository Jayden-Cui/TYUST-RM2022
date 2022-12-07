#include"main/visual_proc.h"
#include"cam_driver/open_camera.h"
#include"armor_detect/armor_detect.h"
#include"usb_serial/serial_usb.h"
#include<thread>
#include<chrono>
#include"time.h"
#include"unistd.h"
#include"sys/types.h"
#include"sys/stat.h"
#include"predict/kala.h"

#define DEBUG_SHOW

static UsbSerial usb_serial;
static string root = "/home/newmakercc/桌面/picgx/";
static string write_path;


void VisualProc::produce()
{
    clock_t start_time, end_time;
    start_time = clock();
    long long n =0;
    int frame_num = 0;

    int break_flag = false;

    for(int i = 1; i < 501; i++)
    {
        string path = root + to_string(i);
        char * path_char = const_cast<char*>(path.c_str());
        int a = access(path_char, 0);

        if(a != -1)
        {
           ;// continue;
        }
        else
        {
            write_path = path;
            mkdir(path_char, 0777);
            break_flag = true;
            break;
        }
    }
    if(break_flag != true)
    {

       // write_path = root + to_string(500);
    }

    usb_serial.SerialInit();
    dvpStr cam_config_path = "/home/newmakercc/camera/44.ini";
    //usb_serial.SerialInit();
    open_camera camera_one;
    camera_one.openFrame(cam_config_path);
    cv::Mat src_image;
    cv::Rect2d roi_rect;


    ArmorProcess Armor;

    cv::RotatedRect rect(cv::Point2f(0,0),cv::Point2f(1280,0),cv::Point2f(1280,1024));
    ArmorPosture fight_info(rect);
    std::vector<armor>final_armor_list;



    while (true)
    {

        //start_time = clock();
        //time = (cv::getTickCount() - time)/cv::getTickFrequency();
        double time = cv::getTickCount();
        src_image = camera_one.GetFrame();
        //src_image = imread("/home/newmakercc/桌面/picgx/89/3940.jpg");
        //cv::resize(src_image,src_image,cv::Size(640,480));
        if(ArmorType != NO_GET)
        {
            fight_info.armor_color = ArmorType;

//1300
//1050.3    MM
#define tuoluoyihigh = 105.03;
           // tuoluoyihigh / fight_info.armor_pitch
            double ttp = -(tP-5.5)* 3.141592/ 180;
            int dish = abs(abs(105 / ( sin(ttp)  )) - fight_info.distance);
          fight_info.tyv =  tyv  ;
              fight_info.Adish = dish;
              roi_rect = Armor.GetRoi(src_image,fight_info);

              Armor.ArmorDetect(src_image,fight_info,usb_serial,roi_rect,dish,tP,tY);

              cout<<"PPPPPdishdishdishdish"<<dish<<endl;

              cout<<"tuoluoyiP"<< tP<<endl;

              cout<<"tuoluoyisin(ttp)"<< sin(ttp)<<endl;

              cout<<"tuoluoyiPv"<< tpv<<endl;

              cout<<"tuoluoyiYv"<< tyv<<endl;



        }

#ifdef DEBUG_SHOW
        time = (cv::getTickCount() - time)/cv::getTickFrequency();
        //cout<<"FPS"<<time*1000<<"ms"<<endl;
        char string[10];
        sprintf(string, "%.2f",fight_info.distance);      // 帧率保留两位小数

        //sprintf(string, "%.2f", time*1000);      // 帧率保留两位小数
        std::string fpsString("Dis:");
        fpsString += string;                    // 在"FPS:"后加入帧率数值字符串
        std::string fpsString1("CM");
        fpsString +=fpsString1;

        char stringC[10];
        sprintf(stringC, "%.2f",fight_info.Adish);      // 帧率保留两位小数

        //sprintf(string, "%.2f", time*1000);      // 帧率保留两位小数
        std::string fpsStringCC("DISH:");
        fpsStringCC += stringC;                    // 在"FPS:"后加入帧率数值字符串
        std::string fpsString1CC("CM");
        fpsStringCC +=fpsString1CC;




        //图像矩阵, string型文字内容 ,文字坐标，以左下角为原点 ,字体类型 ,字体大小,字体颜色,
        cv::putText(src_image, fpsString,cv::Point(5, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        cv::putText(src_image, fpsStringCC,cv::Point(750, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        //cv::putText(src_image, fpsString,cv::Point(5, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        //图像矩阵, string型文字内容 ,文字坐标，以左下角为原点 ,字体类型 ,字体大小,字体颜色,
       // cv::putText(src_image, fpsString,cv::Point(5, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        if(ArmorType == NO_GET)
        {
            std::string str = "NO_COLOR";
            cv::putText(src_image, fpsString,cv::Point(230, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        }
        else if(ArmorType ==BLUE)
        {
            std::string str = "Fight Blue";
            cv::putText(src_image, str,cv::Point(230, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        }
        else
        {
            std::string str = "Fight Red";
            cv::putText(src_image, str,cv::Point(230, 50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0, 255, 0),2);
        }



        imshow("test",src_image);


      //imwrite(write_path + "/" + to_string(n++) + ".jpg", src_image);
        if(cv::waitKey(1) == 'q')
        {
            cv::destroyAllWindows();
            break;
        }
        //cout<<enemy_color<<endl;
#endif
        //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}

/****************************************
* @funcName consumer
* @brief    接收数据线程
* @para     无
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/
void VisualProc::consumer()
{
    //usb_serial.SerialInit();

    int16_t  rec_data[8];
    static int num = 0;
    while (true)
    {
        /************************/
        if(num < 10000)
        {
            num++;
           cout<<"strange"<<endl;
        }
        /*************************/
        else
{
           ArmorType = BLUE;
            int16_t  while_error;
            num = 20000;

            usb_serial.SerialRecData(rec_data);
            tY = rec_data[3]/100.0;
            tP = rec_data[2]/100.0;
            tpv = rec_data[4];
            tyv = rec_data[5];



       if(rec_data[0] == 1)
       {

           if(rec_data[1] == 2)
           {
               ArmorType = BLUE;
           }
           if(rec_data[1] == 1)
           {
               ArmorType = RED;
           }
           if(rec_data[7] == 1 )
           {
//                    tuoluo

        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}}
}
