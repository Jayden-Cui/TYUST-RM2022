#ifndef SERIAL_USB_H
#define SERIAL_USB_H
#include <iostream>
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <usb_serial/CRC_Check.h>
#include <stdlib.h>
#include <cstdio>
#include <unistd.h>
#include <sys/ioctl.h>
#include <main/visual_proc.h>
using namespace std;


#define TRUE 1
#define FALSE 0

//模式
#define CmdID0 0x00; //关闭视觉
#define CmdID1 0x01; //识别红色
#define CmdID2 0x02; //识别蓝色

//字节数为4的结构体
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

typedef struct
{
    float2uchar yaw_angle;//偏航角
    float2uchar pitch_angle;//俯仰角
    float2uchar dis;//目标距离
    int ismiddle ;//设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，目前暂不使用，默认置0
    int isFindTarget;//当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int istuoluokaiguan;
    int nearFace;
    int imuy;
    int imup;
    int imumode1;
    int imumode2;
    int imumode3;
} VisionData;


class UsbSerial
{
public:
    UsbSerial(){};
    ~UsbSerial(){};
    bool SerialInit();
    bool SerialSendData(uint8_t* serial_data);
    void SerialRecData(int16_t* serial_data);
    void TransformData(const VisionData &data);
    void TransformDataFirst(int Xpos, int Ypos, int dis);//方案1
    void ReciveData(const VisionData &data);
    void closePort();
    void send();

private:
    int fd;

    int speed, databits, stopbits, parity;
    unsigned char rdata[255]; //raw_data
    unsigned char Tdata[30];  //transfrom data

};
#endif // SERIAL_USB_H
