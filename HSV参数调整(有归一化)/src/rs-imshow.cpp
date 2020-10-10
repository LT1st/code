
#include <librealsense2/rs.hpp>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>
#include<string>

using namespace std;
using namespace cv;
//输入图像
Mat img;
//灰度值归一化
Mat bgr;
//HSV图像
Mat hsv;
//色相
int hmin = 0;
int hmin_Max = 360;
int hmax = 360;
int hmax_Max = 360;
//饱和度
int smin = 0;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
//亮度
int vmin = 0;
int vmin_Max = 255;
int vmax = 255;
int vmax_Max = 255;
//显示原图的窗口
string windowName = "src";
//输出图像的显示窗口
string dstName = "src";
//输出图像
Mat dst;
//回调函数
void callBack(int, void*)
{
    //输出图像分配内存
    dst = Mat::zeros(img.size(), CV_32FC3);
    //掩码
    Mat mask;
    inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
    //只保留
    for (int r = 0; r < bgr.rows; r++)
    {
        for (int c = 0; c < bgr.cols; c++)
        {
            if (mask.at<uchar>(r, c) == 255)
            {
                dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
            }
        }
    }

    vector<Vec3f> circles;
    Mat dst_s;

    cvtColor(dst,dst,COLOR_BGR2GRAY);
    normalize(dst,dst,1.0,0.0,NORM_MINMAX);//归一到0~1之间 
    dst.convertTo(dst_s, CV_8UC1, 255, 0); //转换为0~255之间的整数 

    HoughCircles(dst_s, circles, HOUGH_GRADIENT, 1.5, 70, 100, 70, 0, 100);

    //【5】依次在图中绘制出圆
    for (size_t i = 0; i < circles.size(); i++)
    {
        //参数定义
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //绘制圆心
        //circle(RGBImg, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        //绘制圆轮廓
        circle(dst_s, center, radius, Scalar(155, 50, 255), 3, 8, 0);


        //绘制圆心
        circle(dst_s, center, 3, Scalar(0, 255, 0), -1, 8, 0);
    }
        //输出图像
    imshow(dstName, dst_s);
}

int main(int argc, char*argv[])
{
    //输入图像
   // img = imread("1.jpg", IMREAD_COLOR);
     img = imread("tst_Color.png", IMREAD_COLOR);
    // if (!img.data || img.channels() != 3)
    //  return -1;

 /*     
        realsense流输入
        //构建一个抽象设备的管道
        rs2::pipeline pipe;
        //使用非默认配置文件创建配置以配置管道
        rs2::config cfg;
        //将所需的流添加到配置
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        //指示管道使用请求的配置开始流式传输
        pipe.start(cfg);
        // 放下几个第一帧以使自动曝光稳定
        rs2::frameset frames;
        for(int i = 0; i < 30; i++)        
        //等待所有配置的流生成一个帧
        frames = pipe.wait_for_frames(15000);        
        //获取每个帧
        rs2::frame color_frame = frames.get_color_frame();
        //从彩色图像创建OpenCV矩阵
        Mat RGBImg(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);    

        imshow(windowName, RGBImg);

        img=RGBImg.clone(); */

    //彩色图像的灰度值归一化
    img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
    //颜色空间转换
    cvtColor(bgr, hsv, COLOR_BGR2HSV);
    //定义输出图像的显示窗口
   // namedWindow(dstName, WINDOW_GUI_EXPANDED);
    namedWindow(dstName, WINDOW_AUTOSIZE);
    //调节色相 H
    createTrackbar("hmin", dstName, &hmin, hmin_Max, callBack);
    createTrackbar("hmax", dstName, &hmax, hmax_Max, callBack);
    //调节饱和度 S
    createTrackbar("smin", dstName, &smin, smin_Max, callBack);
    createTrackbar("smax", dstName, &smax, smax_Max, callBack);
    //调节亮度 V
    createTrackbar("vmin", dstName, &vmin, vmin_Max, callBack);
    createTrackbar("vmax", dstName, &vmax, vmax_Max, callBack);
    callBack(0, 0);
    waitKey(0);
    return 0;
    
}