#include "serial_usb.h"

/****************************************
* @funcName SerialInit
* @brief    打开串口权限
* @para     无
* @return   无
* @date     2021.4.27
* @author   wjl
*****************************************/

bool UsbSerial::SerialInit()
{
    cout<<"+--------------------------------------"<<endl;
       cout<<"|            Usb Serial Send Data     |"<<endl;
       cout<<"+--------------------------------------"<<endl;
       //fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
       fd = open("/dev/ttyUSB0",O_RDWR);
       if(-1 == fd)
       {
           cout<<"Error open Serial"<<endl;
           return false;
       }
       else
       {
           cout<<"Open Serial Successfully"<<endl;
       }
       /*---------- Setting the Attributes of the serial port using termios structure --------- */

       struct termios SerialPortSettings;	/* Create the structure                          */

       tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

       cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
       cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */

       SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
       SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
       SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
       SerialPortSettings.c_cflag |= CS8;       /* Set the data bits = 8                                 */

       SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
       SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


       SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
       SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

       SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

       SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
       if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
       {
           cout<<"\n  ERROR ! in Setting attributes"<<endl;
       }
       else
       {
           cout<<"\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity = none"<<endl;
           cout<<"Serial Start Work "<<endl;
       }
       tcflush(fd,TCIFLUSH);
       cout<<"+--------------------------------------"<<endl;
       /*------------------------------- Write data to serial port -----------------------------*/
       return true;
}



void UsbSerial::TransformData(const VisionData &data)
{

    Tdata[0] = 0xA5;

    Tdata[1] = CmdID1;
    Append_CRC8_Check_Sum(Tdata, 3);

    Tdata[3] = data.pitch_angle.c[0];
    Tdata[4] = data.pitch_angle.c[1];
    Tdata[5] = data.pitch_angle.c[2];
    Tdata[6] = data.pitch_angle.c[3];

    Tdata[7] = data.yaw_angle.c[0];
    Tdata[8] = data.yaw_angle.c[1];
    Tdata[9] = data.yaw_angle.c[2];
    Tdata[10] = data.yaw_angle.c[3];

    Tdata[11] = data.dis.c[0];
    Tdata[12] = data.dis.c[1];
    Tdata[13] = data.dis.c[2];
    Tdata[14] = data.dis.c[3];
    //cout<<"888888888"<<data.dis<<endl;
//    if(data.dis>500)
//    data.ismiddle = 0;
    Tdata[15] = data.ismiddle;
    Tdata[16] = data.isFindTarget;

    Tdata[17] = data.istuoluokaiguan;
    Tdata[18] = 0x00;
    Tdata[19] = data.nearFace;

    Append_CRC16_Check_Sum(Tdata, 22);

}




//发送数据函数
void UsbSerial::send()
{
    write(fd, Tdata, 22);
    std::cout << "OK" << std::endl;

    for(int i = 0; i < 22; i++)
    {
       printf("%xiiii",Tdata[i]);


    }
}


//关闭通讯协议接口
void UsbSerial::closePort()
{
    close(fd);
}



//bool UsbSerial::SerialSendData(uint8_t *serial_data)
//{
//    int  bytes_written  = 0;
//    bytes_written = write(fd,serial_data,10);
//    //std::cout << "OK" << std::endl;

////    for(int i = 0; i < 10; i++)
////    {
////        printf("%x",serial_data[i]);
////    }
//}

void UsbSerial::SerialRecData(int16_t *serial_data)
{



    int bytes_read = 0;
    //bytes_read = read(fd, &read_buffer, 3);


    size_t bytes;
        char *name = ttyname(fd);
        if (name = NULL)printf("tty is null\n");
        if (name != NULL)printf("device:%s\n",name);
        ioctl(fd, FIONREAD, &bytes);
        //if (result == -1)return false;


//        if (bytes == 0)
//        {
//        //    cout << "缓冲区为空" << endl;
//            return true;
//        }

        bytes = read(fd, rdata, 22);

        if (rdata[0] == 0xA5 &&  Verify_CRC8_Check_Sum(rdata, 3)   )
        {

            //判断针头和CRC校验是否正确

             serial_data[0] = (int)rdata[3];//是否开启自瞄
             serial_data[1] = (int)rdata[4];//颜色



             float tuoluoyiP = 0,tuoluoyiY = 0,tuoluoyiPV = 0,tuoluoyiYV = 0;
              //serial_data[3] = ((int16_t)(rdata[5]<<8 | rdata[6]))/100.0;
              tuoluoyiP =((int16_t)(rdata[5]<<8 | rdata[6]))/100.0f; //| (uint32_t)rdata[6]<<8 | (uint32_t)rdata[5]);
              cout<<"tuoluoyiP"<< tuoluoyiP<<endl;
              serial_data[2]  = tuoluoyiP*100.0;


              tuoluoyiY =((int16_t)(rdata[7]<<8 | rdata[8]))/100.0f;
              serial_data[3]  = tuoluoyiY*100.0;
              cout<<"tuoluoyiY"<<tuoluoyiY<<endl;



              tuoluoyiPV =((int16_t)(rdata[9]<<8 | rdata[10]))/100.0f; //| (uint32_t)rdata[6]<<8 | (uint32_t)rd             cout<<"bbbbbbbbbbBBBBB"<<tuoluoyiY<<endl;
              serial_data[4]  = tuoluoyiPV;
              cout<<"tuoluoyiPV"<<tuoluoyiPV<<endl;

              tuoluoyiYV =((int16_t)(rdata[11]<<8 | rdata[12]))/100.0f;
              serial_data[5]  = tuoluoyiYV;
              cout<<"tuoluoyiYV"<<tuoluoyiYV<<endl;

              serial_data[6] = (int)rdata[13];//反陀螺











//    for(int i = 0; i < 22; i++)
//    {
//       printf("%d接受到的数据",rdata[i]);

//    }

}
}
