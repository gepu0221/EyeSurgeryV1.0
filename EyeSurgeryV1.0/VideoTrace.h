#ifndef VIDEOTRACE_H
#define VIDEOTRACE_H

#include<iostream>
#include<fstream>
#include<vector>
#include<cstdlib>
#include "FindEll.h"
#include "EFCHandler.h"
#include<opencv2/opencv.hpp>

#define LOWTHRESHOLD 70
#define MAXTHRESHOLD 200
#define DIST 0.5
#define NUM 5
#define RECTROAT 0.25//局部追踪的边框位置
#define IFSHELETER 1.4//判断是否被遮挡的界线
#define RADII 6 //针脚的半径
#define OFFSETMAX 50//最大偏移距离
#define RESETAREA 4//当被均被遮挡时的区域初始化
//KalmanFilter
#define MAX_OFFSET_PER_FRAME 3//追踪时每帧最大的偏移距离

typedef  unsigned char uchar;
class VideoTrace
{
public:
	int r;
	int c;
	int hc;//half of c(只处理一半）
	long frameNum;// the frame number of the video
	int delay;//the time interval of two frames which are continuous
	cv::VideoCapture captureVideo;//the containter of the vedio
	int video_num;//the number of video in video group
	cv::VideoCapture * captureVideo_g;//the containter of the vedio group which reads directly
	cv::Mat frame;//the containter of the vedio capture
	cv::Mat hframe;//the containter of the vedio capture of half
	cv::Mat rate_frame;//缩放frame_rate比例的frame
	const double frame_rate_r = 0.6;//视频缩小比例
	const double frame_rate_c = 0.5;
	int offset_frame_r, offset_frame_c;

	std::vector<cv::Point> edge;//椭圆的轮廓点集
	std::vector<cv::Point> pset;//椭圆的点集（包括区域内的点）

	//Meanshift
	bool is_tracking;//标记是否进行追踪
	bool pause;
	double *hist1, *hist2;
	double *m_wei;//权值矩阵
	double C = 0.0;//归一化系数
	//拟合椭圆的中心坐标
	cv::Point center;
	//拟合椭圆的长轴和短轴
	int t_w;//短轴
	int t_h;//长轴
	int angle;//椭圆角度
	cv::Mat pic_hist;
	//标准椭圆（用于标记偏移）
	cv::Point center_stad;//标准椭圆中心
	std::vector<cv::Point> label_p_stad;//16等分点位置
	std::vector<cv::Point> label_p_std;//标准的16等分点位置
	cv::RotatedRect ellip;//绘制追踪后的椭圆
	cv::Point p_n[16], p_n_s[16];//记录16个缝针点的内位置
	cv::Point p_no[16], p_no_s[16];//记录16个缝针点的外位置

	//局部追踪
	double *histc1[9];//3*3直方图

	int local_lx, local_ly;
	int local_rx, local_ry;
	CvRect drawing_box;
	int out_lx, out_ly, out_rx, out_ry;
	int in_lx, in_ly, in_rx, in_ry;
	//3*3坐标
	int rt_h, rt_w;//3*3中每一个小矩形的长和宽
	int x[9], x_std[9];//opencv坐标系
	int y[9], y_std[9];//opencv坐标系

	//卡尔曼滤波
	cv::KalmanFilter KF;
	cv::Mat measurement;
	cv::Mat prediction;

	//Test Erroe;
	int point_off;

public:
	VideoTrace(char * filename)
	{
		VideoReadInit(filename);

		double rate_r = (1 - frame_rate_r) / 2;
		double rate_c = (1 - frame_rate_c) / 2;
		int begin_r = r*rate_r, end_r = r - begin_r;
		int begin_c = c*rate_c, end_c = c - begin_c;

		//create a new image of RBG channel
		frame = cv::Mat(r, c, CV_8UC3, cv::Scalar(0));
		hframe = cv::Mat(r, hc, CV_8UC3, cv::Scalar(0));
		rate_frame = cv::Mat((end_r - begin_r + 1), (end_c - begin_c + 1), CV_8UC3, cv::Scalar(0));
		is_tracking = false;
		pause = false;

		//卡尔曼滤波
		//@stateNum=4状态数（x，y，dx，dy）
		//@measureNum=2观测量，能看到的坐标值（x，y）
		KF.init(4, 2, 0, 5);
		//状态转移矩阵
		KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		//观察矩阵（也就是算法追踪到的目标偏移）
		setIdentity(KF.measurementMatrix);
		//状态/系统噪声矩阵
		setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
		//观测噪声矩阵
		setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
		//后验错误估计协方差矩阵
		setIdentity(KF.errorCovPost, cv::Scalar::all(1));
		randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
		measurement = cv::Mat::zeros(2, 1, CV_32F);
		//prediction = Mat::zeros(2, 1, CV_32F);
	}
	//文件读写的构造函数
	VideoTrace(char * filename, char * dstfile)
	{

		VideoReadInit(filename);

		double rate_r = (1 - frame_rate_r) / 2;
		double rate_c = (1 - frame_rate_c) / 2;
		int begin_r = r*rate_r, end_r = r - begin_r;
		int begin_c = c*rate_c, end_c = c - begin_c;
		offset_frame_r = begin_r;
		offset_frame_c = begin_c;

		//create a new image of RBG channel
		frame = cv::Mat(r, c, CV_8UC3, cv::Scalar(0));
		hframe = cv::Mat(r, hc, CV_8UC3, cv::Scalar(0));
		rate_frame = cv::Mat((end_r - begin_r + 1), (end_c - begin_c + 1), CV_8UC3, cv::Scalar(0));
		is_tracking = false;
		pause = false;

		//卡尔曼滤波
		//@stateNum=4状态数（x，y，dx，dy）
		//@measureNum=2观测量，能看到的坐标值（x，y）
		KF.init(4, 2, 0, 5);
		//状态转移矩阵
		KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		//观察矩阵（也就是算法追踪到的目标偏移）
		setIdentity(KF.measurementMatrix);
		//状态/系统噪声矩阵
		setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
		//观测噪声矩阵
		setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
		//后验错误估计协方差矩阵
		setIdentity(KF.errorCovPost, cv::Scalar::all(1));
		randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
		measurement = cv::Mat::zeros(2, 1, CV_32F);
		//prediction = Mat::zeros(2, 1, CV_32F);
	}

	//直接读取摄像头组
	VideoTrace(int _video_num)
	{
		VideoReadInit_Direct();

		double rate_r = (1 - frame_rate_r) / 2;
		double rate_c = (1 - frame_rate_c) / 2;
		int begin_r = r*rate_r, end_r = r - begin_r;
		int begin_c = c*rate_c, end_c = c - begin_c;
		offset_frame_r = begin_r;
		offset_frame_c = begin_c;

		//create a new image of RBG channel
		frame = cv::Mat(r, c, CV_8UC3, cv::Scalar(0));
		hframe = cv::Mat(r, hc, CV_8UC3, cv::Scalar(0));
		rate_frame = cv::Mat((end_r - begin_r + 1), (end_c - begin_c + 1), CV_8UC3, cv::Scalar(0));
		is_tracking = false;
		pause = false;

		//卡尔曼滤波
		//@stateNum=4状态数（x，y，dx，dy）
		//@measureNum=2观测量，能看到的坐标值（x，y）
		KF.init(4, 2, 0, 5);
		//状态转移矩阵
		KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		//观察矩阵（也就是算法追踪到的目标偏移）
		setIdentity(KF.measurementMatrix);
		//状态/系统噪声矩阵
		setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
		//观测噪声矩阵
		setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
		//后验错误估计协方差矩阵
		setIdentity(KF.errorCovPost, cv::Scalar::all(1));
		randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
		measurement = cv::Mat::zeros(2, 1, CV_32F);
		//prediction = Mat::zeros(2, 1, CV_32F);
	}

	~VideoTrace()
	{

	}

	int VideoReadInit(char * filename);

	//直接读取四个摄像头数据
	int VideoReadInit_Direct();

	//获取视频图像的一半（双目的一半）
	void getHalf();

	//获取视频图像的frame_rate倍
	void getFrameRate();

	//获取椭圆点集
	int getEdgeset();

	//获取EFC椭圆点集
	int getEFCEdgeset();

	//获取椭圆轮廓内区域的点集
	void getEll_pointset();


	//获取椭圆轮廓内区域的点集
	void getEll_pointset_myself();

	//标记单个像素点（在原图src上）
	void LabelPixel(int x, int y);

	//获取16个缝针点的初始化位置(ith为第几个点）
	void get_pn();

	//获得一个九等分点（缝针点和椭圆中心之间）
	void get_nine_half(int sort);

	//椭圆的绘制
	void draw_ellp();


	//Meanshift
	//初始化用于绘制的椭圆
	void Init_ell_mesg();

	//在追踪时获得椭圆边缘点集
	void getEllipEdge_tracing();

	//初始化直方图
	void init_target_ellpise(cv::Mat & src);

	void MeanShift_Tracking_ellpise(cv::Mat & src);

	//追踪初始化
	void begin_tracing();

	//获取帧并训练主函数
	void tracing();

	void tracing_choose();

	//单个追踪每一个帧（c#+u3d），返回结果帧
	//@video_index:使用的video_group中的摄像头index
	uchar* tracing_each_frame(int video_index);

	/*局部追踪*/

	//用于局部追踪排除遮挡和污染部分
	void init_w();

	void begin_tracing_choose();

	//初始化中心位置的直方图（3*3中的第4）
	void begin_tracing_choose_cen();

	void getRect_pointset();

	void init_target(cv::Mat & src);

	void MeanShift_Tracking(cv::Mat & src);

	//ith 为在本帧中被遮挡的3*3中的第ith个区域
	int choose_area(int ith);

	void init_target_choose(cv::Mat & src, int ori_x, int ori_y, int ori_w, int ori_h, int ith);


	int MeanShift_Tracking_choose(cv::Mat & src, int ith);

	//初始化用于追踪的边界矩阵
	void Init_ell_traceRect();

	//局部区域的椭圆的绘制
	void draw_ellp_choose(int sort);

	//标记手术的针脚（sort 为0-15中的一个）
	void label_needle(int sort);

	//标记一条直线
	void MyLine(cv::Point start, cv::Point end);


	//用于偏移过大时恢复原位置
	void reset();

	/*卡尔曼滤波*/
	void KF_process(int offset_x, int offset_y);
};
#endif