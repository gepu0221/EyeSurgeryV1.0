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
#define RECTROAT 0.25//�ֲ�׷�ٵı߿�λ��
#define IFSHELETER 1.4//�ж��Ƿ��ڵ��Ľ���
#define RADII 6 //��ŵİ뾶
#define OFFSETMAX 50//���ƫ�ƾ���
#define RESETAREA 4//���������ڵ�ʱ�������ʼ��
//KalmanFilter
#define MAX_OFFSET_PER_FRAME 3//׷��ʱÿ֡����ƫ�ƾ���
enum {
	XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT
};

//��Բ����
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
	int lowThreshold;//Canny ����ֵ
	int maxThreshold;//Canny ����ֵ
	cv::Mat src;//��ȡͼƬ
	cv::Mat dst;//Canny��Ե��ȡ�������
	cv::Mat show;//��ϳ�����Բͼ������ȡ��Բ�㼯
	int r, c;
	//��ȡ����
	const double resize_rate = 0.2;//���¿���ͼƬ��size������ȡ��Ե
	std::vector<std::vector<cv::Point>>  contours;
	std::vector<cv::Vec4i> hierarchy;//????Vec4i
	cv::RotatedRect ellipsemege;//���ڻ�����Բ�������Բ
	std::vector<cv::Point> oripset;//��ͼ����ֱ����ȡ���ĵ㼯��ǰ�������ı߽�ϲ���
	std::vector<cv::Point> dstpset;//������Բ�㼯

	/*��ȡ��Բ��������������ת��Ϊ�ʺϵȷֵĵ㼯*/
	std::vector<cv::Point> elledge;//�������Բ�����㼯
	int all;//��Բ��Ե�ĵ�ĸ���
	cv::Point pedge[4];//����ĸ��ȷֵ�
	int num[4];//��ŵȷ��Ĳ���ÿ���ֵ�ĸ���
	int numsort[4];//��¼�ĸ���ǵ��ڳ�ʼ��Բ�����е�λ��
	std::vector<cv::Point> part_label;

	/*����EFC�ȷ���Բ*/
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

	//�ж��Ƿ���߽�
	int JudgeOutEdge(int c_x, int c_y, int w, int h, int rr, int cc);

	//��Canny��Ե�㷨��ȡ��Ե
	void CannyEdge();

	//�����޸Ĵ�С��Canny��Ե�㷨��ȡ��Ե
	void ResizeCannyEdge();

	//draw the Ellipse
	void EllipseEx_my(cv::Mat& img, cv::Point center, cv::Size axes,
		int angle, int arc_start, int arc_end, std::vector<cv::Point> & v);

	void ellipse_my(cv::Mat& img, const cv::RotatedRect& box, std::vector<cv::Point> &v);

	//Ѱ�ұ߽磨�����Բ��
	int findContour();

	//Ѱ�ұ߽磨�����Բ������õ㼯
	int findContourGetPset();

	//Ѱ�ұ߽磨����EFCHandler�����Բ��
	int ECFfindContour();

	//Ѱ����Բ�����ұ�Ե
	void findtlbr();

	//�����Բλ�õ㼯
	void getEllSet();

	//��ԭͼ�����Բ
	void LabelSrc();

	//ͳ��ÿ���ֵ�ĸ����������Ĳ��֣�������newpet�з����top-left-bottom-right˳�������е�����
	void caculatePoint();

	//���ĵȷֺ��һ�ݼ����ȷ��ķݣ��ȷֵ�k�ݣ���ֻ�����ߣ��ұ�ֱ�ӶԳƹ�ȥ
	void LabelOnePart(int k);

	//��ǵ������ص�
	void LabelPixel(int x, int y);

	//�ҵ��ԳƵ㣨y������ͬ��x���겻ͬ��
	void FindSymm(cv::Point pf, cv::Point & pr);

	//��ȡ��Բ����������ͳ�����
	//t_w ���� t_h����
	void getEllMesg(cv::Point & center, int & angle, int & t_w, int & t_h);

	//SHOW
	void showResult();

	//Process
	int AllProcess();

	//������Բ����Ҷϵ�����ȷ�
	int FindEll::ECFAllProcess();

	//test$$$$$$$
	void showPoint();

	//��һ��ͼ��ȫ����Ϊ0Ϊ����ʾ
	void createBack(cv::Mat & show);

	//��������Ѱ�ұ߽�
	void resetBoundary();


	~FindEll()
	{

	}
};

#endif
