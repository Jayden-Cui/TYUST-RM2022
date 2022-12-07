#include<iostream>
#include<thread>
#include"main/visual_proc.h"
#include<opencv2/opencv.hpp>
#include"usb_serial/serial_usb.h"

using namespace std;
using namespace cv;

int main()
{
    VisualProc visual_proc;
    std::thread ImgProduce(&VisualProc::produce,&visual_proc);
    std::thread ImgConsume(&VisualProc::consumer,&visual_proc);

    ImgProduce.join();
    ImgConsume.join();

    return 0;
}


