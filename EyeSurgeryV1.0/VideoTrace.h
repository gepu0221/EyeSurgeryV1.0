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
#define RECTROAT 0.25//�ֲ�׷�ٵı߿�λ��
#define IFSHELETER 1.4//�ж��Ƿ��ڵ��Ľ���
#define RADII 6 //��ŵİ뾶
#define OFFSETMAX 50//���ƫ�ƾ���
#define RESETAREA 4//���������ڵ�ʱ�������ʼ��
//KalmanFilter
#define MAX_OFFSET_PER_FRAME 3//׷��ʱÿ֡����ƫ�ƾ���

typedef  unsigned char uchar;
class VideoTrace
{
public:
	int r;
	int c;
	int hc;//half of c(ֻ����һ�룩
	long frameNum;// the frame number of the video
	int delay;//the time interval of two frames which are continuous
	cv::VideoCapture captureVideo;//the containter of the vedio
	int video_num;//the number of video in video group
	cv::VideoCapture * captureVideo_g;//the containter of the vedio group which reads directly
	cv::Mat frame;//the containter of the vedio capture
	cv::Mat hframe;//the containter of the vedio capture of half
	cv::Mat rate_frame;//����frame_rate������frame
	const double frame_rate_r = 0.6;//��Ƶ��С����
	const double frame_rate_c = 0.5;
	int offset_frame_r, offset_frame_c;

	std::vector<cv::Point> edge;//��Բ�������㼯
	std::vector<cv::Point> pset;//��Բ�ĵ㼯�����������ڵĵ㣩

	//Meanshift
	bool is_tracking;//����Ƿ����׷��
	bool pause;
	double *hist1, *hist2;
	double *m_wei;//Ȩֵ����
	double C = 0.0;//��һ��ϵ��
	//�����Բ����������
	cv::Point center;
	//�����Բ�ĳ���Ͷ���
	int t_w;//����
	int t_h;//����
	int angle;//��Բ�Ƕ�
	cv::Mat pic_hist;
	//��׼��Բ�����ڱ��ƫ�ƣ�
	cv::Point center_stad;//��׼��Բ����
	std::vector<cv::Point> label_p_stad;//16�ȷֵ�λ��
	std::vector<cv::Point> label_p_std;//��׼��16�ȷֵ�λ��
	cv::RotatedRect ellip;//����׷�ٺ����Բ
	cv::Point p_n[16], p_n_s[16];//��¼16����������λ��
	cv::Point p_no[16], p_no_s[16];//��¼16����������λ��

	//�ֲ�׷��
	double *histc1[9];//3*3ֱ��ͼ

	int local_lx, local_ly;
	int local_rx, local_ry;
	CvRect drawing_box;
	int out_lx, out_ly, out_rx, out_ry;
	int in_lx, in_ly, in_rx, in_ry;
	//3*3����
	int rt_h, rt_w;//3*3��ÿһ��С���εĳ��Ϳ�
	int x[9], x_std[9];//opencv����ϵ
	int y[9], y_std[9];//opencv����ϵ

	//�������˲�
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

		//�������˲�
		//@stateNum=4״̬����x��y��dx��dy��
		//@measureNum=2�۲������ܿ���������ֵ��x��y��
		KF.init(4, 2, 0, 5);
		//״̬ת�ƾ���
		KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		//�۲����Ҳ�����㷨׷�ٵ���Ŀ��ƫ�ƣ�
		setIdentity(KF.measurementMatrix);
		//״̬/ϵͳ��������
		setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
		//�۲���������
		setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
		//����������Э�������
		setIdentity(KF.errorCovPost, cv::Scalar::all(1));
		randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
		measurement = cv::Mat::zeros(2, 1, CV_32F);
		//prediction = Mat::zeros(2, 1, CV_32F);
	}
	//�ļ���д�Ĺ��캯��
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

		//�������˲�
		//@stateNum=4״̬����x��y��dx��dy��
		//@measureNum=2�۲������ܿ���������ֵ��x��y��
		KF.init(4, 2, 0, 5);
		//״̬ת�ƾ���
		KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		//�۲����Ҳ�����㷨׷�ٵ���Ŀ��ƫ�ƣ�
		setIdentity(KF.measurementMatrix);
		//״̬/ϵͳ��������
		setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
		//�۲���������
		setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
		//����������Э�������
		setIdentity(KF.errorCovPost, cv::Scalar::all(1));
		randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
		measurement = cv::Mat::zeros(2, 1, CV_32F);
		//prediction = Mat::zeros(2, 1, CV_32F);
	}

	//ֱ�Ӷ�ȡ����ͷ��
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

		//�������˲�
		//@stateNum=4״̬����x��y��dx��dy��
		//@measureNum=2�۲������ܿ���������ֵ��x��y��
		KF.init(4, 2, 0, 5);
		//״̬ת�ƾ���
		KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		//�۲����Ҳ�����㷨׷�ٵ���Ŀ��ƫ�ƣ�
		setIdentity(KF.measurementMatrix);
		//״̬/ϵͳ��������
		setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
		//�۲���������
		setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
		//����������Э�������
		setIdentity(KF.errorCovPost, cv::Scalar::all(1));
		randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
		measurement = cv::Mat::zeros(2, 1, CV_32F);
		//prediction = Mat::zeros(2, 1, CV_32F);
	}

	~VideoTrace()
	{

	}

	int VideoReadInit(char * filename);

	//ֱ�Ӷ�ȡ�ĸ�����ͷ����
	int VideoReadInit_Direct();

	//��ȡ��Ƶͼ���һ�루˫Ŀ��һ�룩
	void getHalf();

	//��ȡ��Ƶͼ���frame_rate��
	void getFrameRate();

	//��ȡ��Բ�㼯
	int getEdgeset();

	//��ȡEFC��Բ�㼯
	int getEFCEdgeset();

	//��ȡ��Բ����������ĵ㼯
	void getEll_pointset();


	//��ȡ��Բ����������ĵ㼯
	void getEll_pointset_myself();

	//��ǵ������ص㣨��ԭͼsrc�ϣ�
	void LabelPixel(int x, int y);

	//��ȡ16�������ĳ�ʼ��λ��(ithΪ�ڼ����㣩
	void get_pn();

	//���һ���ŵȷֵ㣨��������Բ����֮�䣩
	void get_nine_half(int sort);

	//��Բ�Ļ���
	void draw_ellp();


	//Meanshift
	//��ʼ�����ڻ��Ƶ���Բ
	void Init_ell_mesg();

	//��׷��ʱ�����Բ��Ե�㼯
	void getEllipEdge_tracing();

	//��ʼ��ֱ��ͼ
	void init_target_ellpise(cv::Mat & src);

	void MeanShift_Tracking_ellpise(cv::Mat & src);

	//׷�ٳ�ʼ��
	void begin_tracing();

	//��ȡ֡��ѵ��������
	void tracing();

	void tracing_choose();

	//����׷��ÿһ��֡��c#+u3d�������ؽ��֡
	//@video_index:ʹ�õ�video_group�е�����ͷindex
	uchar* tracing_each_frame(int video_index);

	/*�ֲ�׷��*/

	//���ھֲ�׷���ų��ڵ�����Ⱦ����
	void init_w();

	void begin_tracing_choose();

	//��ʼ������λ�õ�ֱ��ͼ��3*3�еĵ�4��
	void begin_tracing_choose_cen();

	void getRect_pointset();

	void init_target(cv::Mat & src);

	void MeanShift_Tracking(cv::Mat & src);

	//ith Ϊ�ڱ�֡�б��ڵ���3*3�еĵ�ith������
	int choose_area(int ith);

	void init_target_choose(cv::Mat & src, int ori_x, int ori_y, int ori_w, int ori_h, int ith);


	int MeanShift_Tracking_choose(cv::Mat & src, int ith);

	//��ʼ������׷�ٵı߽����
	void Init_ell_traceRect();

	//�ֲ��������Բ�Ļ���
	void draw_ellp_choose(int sort);

	//�����������ţ�sort Ϊ0-15�е�һ����
	void label_needle(int sort);

	//���һ��ֱ��
	void MyLine(cv::Point start, cv::Point end);


	//����ƫ�ƹ���ʱ�ָ�ԭλ��
	void reset();

	/*�������˲�*/
	void KF_process(int offset_x, int offset_y);
};
#endif