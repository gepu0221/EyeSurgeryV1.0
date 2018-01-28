#ifndef FINDELL_H
#define FINDELL_H

#include<iostream>
#include<fstream>
#include<vector>
#include<cstdlib>
#include "EFCHandler.h"
#include<opencv2/opencv.hpp>
//30/220
#define LOWTHRESHOLD 150
#define MAXTHRESHOLD 250
#define DIST 0.5
#define NUM 5
#define RECTROAT 0.25//局部追踪的边框位置
#define IFSHELETER 1.4//判断是否被遮挡的界线
#define RADII 6 //针脚的半径
#define OFFSETMAX 50//最大偏移距离
#define RESETAREA 4//当被均被遮挡时的区域初始化
//KalmanFilter
#define MAX_OFFSET_PER_FRAME 3//追踪时每帧最大的偏移距离
enum {
	XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT
};

//椭圆参数
struct EllipsePara
{
	cv::Point2f c;
	float A;
	float B;
	float C;
	float F;

};


class FindEll
{
public:
	int lowThreshold;//Canny 低阈值
	int maxThreshold;//Canny 高阈值
	cv::Mat src;//读取图片
	cv::Mat dst;//Canny边缘提取后的容器
	cv::Mat show;//拟合出的椭圆图用于提取椭圆点集
	int r, c;
	//提取轮廓
	const double resize_rate = 0.2;//重新控制图片的size用于提取边缘
	std::vector<std::vector<cv::Point>>  contours;
	std::vector<cv::Vec4i> hierarchy;//????Vec4i
	cv::RotatedRect ellipsemege;//用于绘制椭圆，拟合椭圆
	std::vector<cv::Point> oripset;//在图像中直接提取出的点集（前两个最大的边界合并）
	std::vector<cv::Point> dstpset;//生成椭圆点集

	/*提取椭圆的轮廓，并将其转化为适合等分的点集*/
	std::vector<cv::Point> elledge;//最初的椭圆轮廓点集
	int all;//椭圆边缘的点的个数
	cv::Point pedge[4];//标记四个等分点
	int num[4];//存放等分四部分每部分点的个数
	int numsort[4];//记录四个标记点在初始椭圆轮廓中的位置
	std::vector<cv::Point> part_label;

	/*利用EFC等分椭圆*/
	const int part_num = 16;
	const int coef_num = 30;


public:
	FindEll(int rr, int cc)
	{
		lowThreshold = LOWTHRESHOLD;
		maxThreshold = MAXTHRESHOLD;
		r = rr;
		c = cc;

		src = cv::Mat(r, c, CV_8UC3, cv::Scalar(0));
		show = cv::Mat(src.rows, src.cols, CV_8U, cv::Scalar(0));
	}

	FindEll()
	{
		lowThreshold = LOWTHRESHOLD;
		maxThreshold = MAXTHRESHOLD;

		src = cv::imread("img00000.bmp");
		if (!src.data)
		{
			std::cout << "Read image error" << std::endl;
		}

		else
		{
			//edo.Init(src.cols, src.rows);
			show = cv::Mat(src.rows, src.cols, CV_8U, cv::Scalar(0));
		}

	}

	//判断是否出边界
	int JudgeOutEdge(int c_x, int c_y, int w, int h, int rr, int cc);

	//用Canny边缘算法提取边缘
	void CannyEdge();

	//重新修改大小用Canny边缘算法提取边缘
	void ResizeCannyEdge();

	//draw the Ellipse
	void EllipseEx_my(cv::Mat& img, cv::Point center, cv::Size axes,
		int angle, int arc_start, int arc_end, std::vector<cv::Point> & v);

	void ellipse_my(cv::Mat& img, const cv::RotatedRect& box, std::vector<cv::Point> &v);

	//寻找边界（拟合椭圆）
	int findContour();

	//寻找边界（拟合椭圆）并获得点集
	int findContourGetPset();

	//寻找边界（利用EFCHandler拟合椭圆）
	int ECFfindContour();

	//寻找椭圆的左右边缘
	void findtlbr();

	//获得椭圆位置点集
	void getEllSet();

	//在原图标记椭圆
	void LabelSrc();

	//统计每部分点的个数（共分四部分），并在newpet中放入从top-left-bottom-right顺序新排列的序列
	void caculatePoint();

	//将四等分后的一份继续等分四份（等分第k份），只标记左边，右边直接对称过去
	void LabelOnePart(int k);

	//标记单个像素点
	void LabelPixel(int x, int y);

	//找到对称点（y坐标相同，x坐标不同）
	void FindSymm(cv::Point pf, cv::Point & pr);

	//获取椭圆的中心坐标和长短轴
	//t_w 短轴 t_h长轴
	void getEllMesg(cv::Point & center, int & angle, int & t_w, int & t_h);

	//SHOW
	void showResult();

	//Process
	int AllProcess();

	//利用椭圆傅里叶系数做等分
	int FindEll::ECFAllProcess();

	//test$$$$$$$
	void showPoint();

	//将一个图像全部设为0为了显示
	void createBack(cv::Mat & show);

	//用于重新寻找边界
	void resetBoundary();


	~FindEll()
	{

	}
};

#endif
