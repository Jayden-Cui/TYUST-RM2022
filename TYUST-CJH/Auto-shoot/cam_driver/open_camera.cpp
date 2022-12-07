#include <cam_driver/DVPCamera.h>
#include <cam_driver/open_camera.h>

/****************************************
* @funcName openFrame
* @brief    使用度申SDK,打开摄像头
* @para     dvpStr camera_congig_file
* @return   无
* @date     2021.4.27
* @author   tjk
*****************************************/
bool open_camera::openFrame(dvpStr camera_congig_file)
{
    status = dvpRefresh(&n);
    if (status == DVP_STATUS_OK)
    {
        // 枚举最多16台相机的信息
       {
            if (n > 16)
            n = 16;
        }
        cout <<"有"<<n<<"台相机"<< endl;
        for (i = 0; i < n; i++)
            // 逐个枚举出每个相机的信息
        {
            status = dvpEnum(i, &info[i]);
            if (status != DVP_STATUS_OK)
            {
               cout << "枚举NO"  << endl;
            }
            else
            {
               cout <<"枚举OK"<< endl;
            }
        }
    }
    status = dvpOpenByName(info[0].FriendlyName,OPEN_NORMAL,&m_handle);
    if (status == DVP_STATUS_OK)
    {
        cout<<"ok!!!"<<endl;//// QMessageBox::about(NULL,"About","Open the camera fail!");
    }
    status = dvpGetStreamState(m_handle,&state);
    if (status != DVP_STATUS_OK)
    {
        cout<<"Get the stream state fail!"<<endl; //QMessageBox::about(NULL,"About","Get the stream state fail!");
    }
    if (state == STATE_STOPED)
    {
        status = dvpGetTriggerState(m_handle,&bTrigStatus);
        if (status != DVP_STATUS_FUNCTION_INVALID)
        {
            // 在启动视频流之前先设置为触发模式
            status = dvpSetTriggerState(m_handle,SoftTriggerFlag ? true : false);
            if (status != DVP_STATUS_OK)
            {
                 //QMessageBox::about(NULL,"About","Set status of trigger fail!");
            }
            else
            {
                cout<<"Set status of trigger"<<endl;
            }
        }
        else
        {
            cout<<"sss"<<endl;//ui->groupBox_trigger->setEnabled(false);
        }

        status = dvpStart(m_handle);
        if (status != DVP_STATUS_OK)
        {
        }
    }
    dvpStr path= camera_congig_file;
    //dvpStr path1= "/home/newmaker/rm_task/Test/test/1280_1024.ini";
    dvpLoadConfig(m_handle,path);
    return true;
 }

/****************************************
* @funcName GetFrame
* @brief    得到图像
* @para     无
* @return   无
* @date     2021.4.27
* @author   tjk
*****************************************/
cv::Mat open_camera::GetFrame()
{
    cv::Mat mat_src;
    status = dvpGetFrame(m_handle, &m_pFrame, &pBuffer,30000);
    mat_src = cv::Mat(m_pFrame.iHeight, m_pFrame.iWidth, CV_8UC3, pBuffer);
    return mat_src;
}
