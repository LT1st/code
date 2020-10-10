
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

//因为进行8位存储，所以除以2
const int max_value_H = 360/2; 
const int max_value = 255; 

const cv::String window_capture_name = "Video Capture";
const cv::String window_detection_name = "Object Detection";

int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;


//回调函数
static void on_low_H_thresh_trackvbar(int, void*)
{
	low_H = min(high_H-1,low_H);//防止最大值小于最小值
	setTrackbarPos("Low H",window_detection_name,low_H);
}


static void on_high_H_thresh_trackbar(int, void*)
{
	high_H = max(high_H, low_H+1);
	setTrackbarPos("High H",window_detection_name,high_H);
}

static void on_low_S_thresh_trackbar(int, void*)
{
	low_S = min(high_S-1, low_S);
	setTrackbarPos("Low S",window_detection_name,low_S);
}

static void on_high_S_thresh_trackbar(int, void*)
{
	high_H = max(high_S,low_S+1);
	setTrackbarPos("High S",window_detection_name,high_S);
}

static void on_low_V_thresh_trackbar(int, void*)
{
	low_V = min(high_V-1, low_V);
	setTrackbarPos("Low V",window_detection_name,low_V);
	
}


static void on_high_V_thresh_trackvar(int, void*)
{
	high_V = max(high_V,low_V+1);
	setTrackbarPos("High V",window_detection_name,high_V);

}


int main(void)
{
	
/* 	VideoCapture cap; //打开摄像头
    cap.open(0);
	namedWindow(window_capture_name);
	namedWindow(window_detection_name);
 */
    namedWindow(window_capture_name);
	namedWindow(window_detection_name);

	createTrackbar("Low H",window_detection_name,&low_H,max_value_H,on_high_H_thresh_trackbar);
	createTrackbar("High H",window_detection_name,&high_H,max_value_H,on_high_H_thresh_trackbar);

	createTrackbar("Low S",window_detection_name,&low_S,max_value,on_low_S_thresh_trackbar);
	createTrackbar("High S",window_detection_name,&high_S,max_value,on_high_S_thresh_trackbar);

	createTrackbar("Low V",window_detection_name,&low_V,max_value,on_low_V_thresh_trackbar);
    createTrackbar("High V",window_detection_name,&high_V,max_value,on_high_V_thresh_trackvar);
	
	Mat frame, frame_HSV, frame_threshold;
	while(true)
	{
		//cap >> frame;   //捕获的每帧给frame
        Mat frame = imread("白球002_Color.png");
		if(frame.empty())
		{
			break;
		}

		cv::cvtColor(frame,frame_HSV,cv::COLOR_BGR2HSV); //转化到HSV空间

//进行阈值操作，输出图像为CV_8UC1
		inRange(frame_HSV,Scalar(low_H,low_S,low_V),Scalar(high_H,high_S,high_V),frame_threshold);

		imshow(window_capture_name,frame);
		imshow(window_detection_name,frame_threshold);

		char key=(char)waitKey(30); //每等待30ms，如果是ESC，退出
		if(key == 20)
			break;
	}
	return 0;
};

