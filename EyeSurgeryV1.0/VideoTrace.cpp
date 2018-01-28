//#include "edopencv.h"
#include "VideoTrace.h"
#include "FindEll.h"
#include "EFCHandler.h"

int VideoTrace::VideoReadInit(char * filename)
{
	captureVideo = cv::VideoCapture(filename);

	//test if open sucessfully,isOpened return true when open sucessfully
	if (!captureVideo.read(frame))
	{
		std::cout << "Error!!Fail to open!" << std::endl;
		return 0;
	}
	//support every frame

	captureVideo >> frame;

	//get the rows number of one frame
	r = frame.rows;
	//get the cols number of one frame
	c = frame.cols;

	hc = c / 2;

	//the time interval of two frames which are continuous
	delay = 30;
	//Get the frame number of the video
	frameNum = captureVideo.get(CV_CAP_PROP_FRAME_COUNT);

	local_lx = 37;
	local_ly = 72;
	local_rx = 77;
	local_ry = 100;
	//���ŵ���x�ᣬ���ŵ���y��
	drawing_box.x = 37;
	drawing_box.y = 72;
	//height��x�ᣬwidth��y��
	drawing_box.width = 77 - 37;
	drawing_box.height = 100 - 72;

	return 1;
}

//ֱ�Ӷ�ȡ�ĸ�����ͷ����
int VideoTrace::VideoReadInit_Direct()
{
	//��ʼ������ͷ��
	captureVideo_g = new cv::VideoCapture(video_num);
	for (int i = 0; i < video_num; i++)
	{
		captureVideo_g[i] = cv::VideoCapture(i);
		if (!captureVideo_g[i].read(frame))
		{
			std::cout << "Error!!Fail to open!" << std::endl;
			return 0;
		}
	}
	//captureVideo_g[0] = cv::VideoCapture("9.mp4");
	captureVideo_g[0] >> frame;

	//get the rows number of one frame
	r = frame.rows;
	//get the cols number of one frame
	c = frame.cols;

	hc = c / 2;

	//the time interval of two frames which are continuous
	delay = 30;
	//Get the frame number of the video
	frameNum = captureVideo.get(CV_CAP_PROP_FRAME_COUNT);

	local_lx = 37;
	local_ly = 72;
	local_rx = 77;
	local_ry = 100;
	//���ŵ���x�ᣬ���ŵ���y��
	drawing_box.x = 37;
	drawing_box.y = 72;
	//height��x�ᣬwidth��y��
	drawing_box.width = 77 - 37;
	drawing_box.height = 100 - 72;

	return 1;
}


//��ȡ��Ƶͼ���һ�루˫Ŀ��һ�룩
void VideoTrace::getHalf()
{
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < hc; j++)
		{
			hframe.at<cv::Vec3b>(i, j)[0] = frame.at<cv::Vec3b>(i, j)[0];
			hframe.at<cv::Vec3b>(i, j)[1] = frame.at<cv::Vec3b>(i, j)[1];
			hframe.at<cv::Vec3b>(i, j)[2] = frame.at<cv::Vec3b>(i, j)[2];
		}
	}
}

//��ȡ��Ƶͼ���frame_rate��
void VideoTrace::getFrameRate()
{
	double rate_r = (1 - frame_rate_r) / 2;
	double rate_c = (1 - frame_rate_c) / 2;
	int begin_r = r*rate_r, end_r = r - begin_r;
	int begin_c = c*rate_c, end_c = c - begin_c;

	for (int i = begin_r; i < end_r; i++)
	{
		for (int j = begin_c; j < end_c; j++)
		{
			rate_frame.at<cv::Vec3b>(i - begin_r, j - begin_c)[0] = frame.at<cv::Vec3b>(i, j)[0];
			rate_frame.at<cv::Vec3b>(i - begin_r, j - begin_c)[1] = frame.at<cv::Vec3b>(i, j)[1];
			rate_frame.at<cv::Vec3b>(i - begin_r, j - begin_c)[2] = frame.at<cv::Vec3b>(i, j)[2];
		}
	}
}

//��ȡ��Բ����������ĵ㼯
void VideoTrace::getEll_pointset()
{
	int re;
	pset.clear();
	for (int i = 0; i <r; i++)
	{
		for (int j = 0; j < hc; j++)
		{
			//r<0��֤���ĵ��ڱ߽�edge��
			if ((re = cv::pointPolygonTest(edge, cv::Point2f(i, j), true)) >= 0)
			{
				pset.push_back(cv::Point2f(i, j));
			}
		}
	}
}

void VideoTrace::getRect_pointset()
{
	for (int i = local_lx; i < local_rx; i++)
	{
		for (int j = local_ly; j < local_ry; j++)
		{
			pset.push_back(cv::Point2f(i, j));
		}
	}
}

//��ȡ��Բ����������ĵ㼯
void VideoTrace::getEll_pointset_myself()
{
	int re;
	pset.clear();
	int t_x = center.x, t_y = center.y, tmpx, tmpy;
	int sz = t_w / 2, lz = t_h / 2;
	int sum, res;
	//�����ƽ���ͳ����ƽ��
	sz = sz*sz;
	lz = lz*lz;
	sum = sz*lz;
	for (int i = 0; i <r; i++)
	{
		for (int j = 0; j < hc; j++)
		{
			tmpx = t_x - i;
			tmpy = t_y - j;
			tmpx = tmpx*tmpx*lz;
			tmpy = tmpy*tmpy*sz;
			res = tmpx + tmpy;
			if (res <= sum)
			{
				pset.push_back(cv::Point2f(i, j));
			}
		}
	}
}

//��ȡ��Բ�㼯
int VideoTrace::getEdgeset()
{
	int re = 0;
	getHalf();
	getFrameRate();

	//20171126
	FindEll fe(r, hc);
	fe.src = hframe;
	re = fe.AllProcess();

	/*FindEll fe(r, c);
	fe.src = frame;
	re = fe.AllProcess();*/
	if (re == 0)
	{
		return re;
	}
	else
	{
		edge = fe.elledge;
		fe.getEllMesg(center, angle, t_w, t_h);
		label_p_stad = fe.part_label;
		label_p_std = fe.part_label;
		center_stad = center;
		getEll_pointset();
		Init_ell_mesg();
		get_pn();

		int y = center.x;
		int x = center.y;

		return re;
	}
}

//��ȡEFC��Բ�㼯
int VideoTrace::getEFCEdgeset()
{
	int re = 0;
	getHalf();
	getFrameRate();
	cv::imwrite("frame_rate.bmp", rate_frame);

	//20171126
	/*FindEll fe(r, hc);
	fe.src = hframe;
	re = fe.ECFAllProcess();

	FindEll fe(r, c);
	fe.src = frame;
	re = fe.ECFAllProcess();*/

	FindEll fe(rate_frame.rows, rate_frame.cols);
	fe.src = rate_frame;
	re = fe.ECFAllProcess();
	if (re == 0)
	{
		return re;
	}
	else
	{
		edge = fe.elledge;
		fe.getEllMesg(center, angle, t_w, t_h);
		label_p_stad = fe.part_label;
		label_p_std = fe.part_label;

		/*rate_frame*/
		center.x += offset_frame_c;
		center.y += offset_frame_r;
		for (int i = 0; i < label_p_stad.size(); i++)
		{
			label_p_stad[i].x += offset_frame_c;
			label_p_stad[i].y += offset_frame_r;
			label_p_std[i].x += offset_frame_c;
			label_p_std[i].y += offset_frame_r;
		}
		/*rate_frame*/

		center_stad = center;
		pset = fe.dstpset;
		Init_ell_mesg();
		get_pn();

		int y = center.x;
		int x = center.y;

		return re;
	}
}

//��׷��ʱ�����Բ��Ե�㼯
void VideoTrace::getEllipEdge_tracing()
{
	edge.clear();
	cv::Point pt;
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < hc; j++)
		{
			if ((int)frame.at<cv::Vec3b>(i, j)[0] == 255 && (int)frame.at<cv::Vec3b>(i, j)[1] == 0 && (int)frame.at<cv::Vec3b>(i, j)[2] == 0)
			{
				pt.x = j;
				pt.y = i;
				edge.push_back(pt);
			}
		}
	}
}

//��ǵ������ص㣨��ԭͼsrc�ϣ�
void VideoTrace::LabelPixel(int x, int y)
{
	frame.at<cv::Vec3b>(y, x)[0] = frame.at<cv::Vec3b>(y - 1, x)[0] = frame.at<cv::Vec3b>(y + 1, x)[0] = frame.at<cv::Vec3b>(y, x + 1)[0] = frame.at<cv::Vec3b>(y, x - 1)[0] = 0;
	frame.at<cv::Vec3b>(y - 1, x - 1)[0] = frame.at<cv::Vec3b>(y + 1, x - 1)[0] = frame.at<cv::Vec3b>(y - 1, x + 1)[0] = frame.at<cv::Vec3b>(y + 1, x + 1)[0] = 0;
	frame.at<cv::Vec3b>(y, x)[1] = frame.at<cv::Vec3b>(y - 1, x)[1] = frame.at<cv::Vec3b>(y + 1, x)[1] = frame.at<cv::Vec3b>(y, x + 1)[1] = frame.at<cv::Vec3b>(y, x - 1)[1] = 255;
	frame.at<cv::Vec3b>(y - 1, x - 1)[1] = frame.at<cv::Vec3b>(y + 1, x - 1)[1] = frame.at<cv::Vec3b>(y - 1, x + 1)[1] = frame.at<cv::Vec3b>(y + 1, x + 1)[1] = 255;
	frame.at<cv::Vec3b>(y, x)[2] = frame.at<cv::Vec3b>(y - 1, x)[2] = frame.at<cv::Vec3b>(y + 1, x)[2] = frame.at<cv::Vec3b>(y, x + 1)[2] = frame.at<cv::Vec3b>(y, x - 1)[2] = 0;
	frame.at<cv::Vec3b>(y - 1, x - 1)[2] = frame.at<cv::Vec3b>(y + 1, x - 1)[2] = frame.at<cv::Vec3b>(y + 1, x + 1)[2] = frame.at<cv::Vec3b>(y + 1, x + 1)[2] = 0;
}

//���һ���ŵȷֵ㣨��������Բ����֮�䣩
void VideoTrace::get_nine_half(int sort)
{
	p_n_s[sort].x = p_n[sort].x = (label_p_stad[sort].x * 9 + ellip.center.x) / 10;
	p_n_s[sort].y = p_n[sort].y = (label_p_stad[sort].y * 9 + ellip.center.y) / 10;
	p_no_s[sort].x = p_no[sort].x = 2 * label_p_stad[sort].x - p_n[sort].x;
	p_no_s[sort].y = p_no[sort].y = 2 * label_p_stad[sort].y - p_n[sort].y;


}

//��ȡ16�������ĳ�ʼ��λ��
void VideoTrace::get_pn()
{

	for (int i = 0; i < 16; i++)
	{
		get_nine_half(i);
	}
	point_off = ellip.center.x - p_n[0].x;
}

//Meanshift
//��ʼ�����ڻ��Ƶ���Բ
void VideoTrace::Init_ell_mesg()
{
	ellip.center = center;
	ellip.angle = angle;
	ellip.size.width = t_w;
	ellip.size.height = t_h;
	KF.statePre.at<float>(0) = KF.statePost.at<float>(0) = ellip.center.x;
	KF.statePre.at<float>(1) = KF.statePost.at<float>(1) = ellip.center.y;
	KF.statePre.at<float>(2) = KF.statePost.at<float>(2) = 0;
	KF.statePre.at<float>(3) = KF.statePost.at<float>(3) = 0;

}

//��ʼ������׷�ٵı߽����
void VideoTrace::Init_ell_traceRect()
{
	int tmpy = t_h*(0.5 + RECTROAT);
	int tmpx = t_w*(0.5 + RECTROAT);
	out_ly = center.y - tmpy;
	out_lx = center.x - tmpx;
	out_ry = center.y + tmpy;
	out_rx = center.x + tmpx;

	//���ŵ���x���꣬���ŵ���y����
	//height��x�ᣬwidth��y��
	rt_w = tmpx * 2 / 3;
	rt_h = tmpy * 2 / 3;
	drawing_box.width = rt_w;
	drawing_box.height = rt_h;

	//Ϊ3*3���긳ֵ
	x[0] = x[3] = x[6] = out_lx;
	y[0] = y[1] = y[2] = out_ly;
	x[1] = x[4] = x[7] = out_lx + drawing_box.width;
	y[3] = y[4] = y[5] = out_ly + drawing_box.height;
	x[2] = x[5] = x[8] = out_lx + drawing_box.width + drawing_box.width;
	y[6] = y[7] = y[8] = out_ly + drawing_box.height + drawing_box.height;

	x_std[0] = x_std[3] = x_std[6] = out_lx;
	y_std[0] = y_std[1] = y_std[2] = out_ly;
	x_std[1] = x_std[4] = x_std[7] = out_lx + drawing_box.width;
	y_std[3] = y_std[4] = y_std[5] = out_ly + drawing_box.height;
	x_std[2] = x_std[5] = x_std[8] = out_lx + drawing_box.width + drawing_box.width;
	y_std[6] = y_std[7] = y_std[8] = out_ly + drawing_box.height + drawing_box.height;

	local_lx = x[0];
	local_ly = y[0];
	drawing_box.y = y[0];
	drawing_box.x = x[0];

}

//����ƫ�ƹ���ʱ�ָ�ԭλ��
void VideoTrace::reset()
{
	//cout << "begin_reset" << endl;
	ellip.center = center_stad;
	for (int i = 0; i < 9; i++)
	{
		x[i] = x_std[i];
		y[i] = y_std[i];
		p_n[i].x = p_n_s[i].x;
		p_n[i].y = p_n_s[i].y;
		p_no[i].x = p_no_s[i].x;
		p_no[i].y = p_no_s[i].y;
	}
	for (int i = 9; i < 16; i++)
	{
		p_n[i].x = p_n_s[i].x;
		p_n[i].y = p_n_s[i].y;
		p_no[i].x = p_no_s[i].x;
		p_no[i].y = p_no_s[i].y;
	}
	label_p_stad = label_p_std;
	int ith = RESETAREA;
	//%%%%%%%%%%%%%%
	drawing_box.x = x[ith];
	drawing_box.y = y[ith];
	drawing_box.width = rt_w;
	drawing_box.height = rt_h;

}

void VideoTrace::init_target_ellpise(cv::Mat & src)
{
	int t_x, t_y;
	int areac;//׷����������ص����
	double h, dist;
	int i, j;
	int x, y;
	int q_r, q_g, q_b, q_temp;
	cv::Point ptmp;

	t_x = center.x;
	t_y = center.y;
	areac = pset.size();

	h = t_h*t_h;//��������Ϊ��Բ�ĳ���
	pic_hist = cv::Mat(200, 300, CV_8U, cv::Scalar(0)); //����ֱ��ͼͼ��

	//��ʼ��Ȩֵ�����Ŀ��ֱ��ͼ
	for (i = 0; i < areac; i++)
	{
		m_wei[i] = 0.0;
	}

	for (i = 0; i<4096; i++)
	{
		hist1[i] = 0.0;
	}

	for (i = 0; i < areac; i++)
	{
		ptmp = pset[i];
		dist = pow(ptmp.x - center.x, 2) + pow(ptmp.y - center.y, 2);
		m_wei[i] = 1 - dist / h;
		C += m_wei[i];
	}

	//����Ŀ��Ȩֱֵ��
	for (i = 0; i < areac; i++)
	{
		ptmp = pset[i];
		y = ptmp.x;
		x = ptmp.y;
		//rgb��ɫ�ռ�����Ϊ16*16*16 bins
		//rgb��ɫ�ռ�����Ϊ16*16*16 bins
		q_r = ((unsigned char)src.at<cv::Vec3b>(x, y)[2]) / 16;
		q_g = ((unsigned char)src.at<cv::Vec3b>(x, y)[1]) / 16;
		q_b = ((unsigned char)src.at<cv::Vec3b>(x, y)[0]) / 16;
		q_temp = q_r * 256 + q_g * 16 + q_b;
		hist1[q_temp] = hist1[q_temp] + m_wei[i];
	}

	//��һ��ֱ��ͼ
	for (i = 0; i<4096; i++)
	{
		hist1[i] = hist1[i] / C;
	}

	//����Ŀ��ֱ��ͼ
	double temp_max = 0.0;

	for (i = 0; i < 4096; i++)			//��ֱ��ͼ���ֵ��Ϊ�˹�һ��
	{
		//printf("%f\n",val_hist[i]);
		if (temp_max < hist1[i])
		{
			temp_max = hist1[i];
		}
	}
	//��ֱ��ͼ
	cv::Point p1, p2;
	double bin_width = (double)pic_hist.cols / 4096;
	double bin_unith = (double)pic_hist.rows / temp_max;

	for (i = 0; i < 4096; i++)
	{
		p1.x = i * bin_width;
		p1.y = pic_hist.rows;
		p2.x = (i + 1)*bin_width;
		p2.y = pic_hist.rows - hist1[i] * bin_unith;
		rectangle(pic_hist, p1, p2, cvScalar(0, 255, 0), -1, 8, 0);
	}
	imwrite("hist1.jpg", pic_hist);
	pic_hist.release();
}

void VideoTrace::MeanShift_Tracking_ellpise(cv::Mat & src)
{
	int num = 0, i = 0, j = 0;
	//int t_w = 0, t_h = 0, t_x = 0, t_y = 0;
	int  t_x = 0, t_y = 0;
	double *w = 0, *hist2 = 0;
	double sum_w = 0, x1 = 0, x2 = 0, y1 = 2.0, y2 = 2.0;
	int q_r, q_g, q_b;
	int *q_temp;
	cv::Mat pic_hist;
	int r, count, x, y;
	cv::Point ptmp;

	count = pset.size();

	pic_hist = cv::Mat(200, 300, CV_8U, cv::Scalar(0)); //����ֱ��ͼͼ��
	hist2 = (double *)malloc(sizeof(double)* 4096);
	w = (double *)malloc(sizeof(double)* 4096);
	q_temp = (int *)malloc(sizeof(int)*count);

	while ((pow(y2, 2) + pow(y1, 2) > 0.5) && (num < NUM))
	{
		num++;
		t_x = center.x;
		t_y = center.y;
		memset(q_temp, 0, sizeof(int)*count);
		for (i = 0; i<4096; i++)
		{
			w[i] = 0.0;
			hist2[i] = 0.0;
		}

		for (i = 0; i < count; i++)
		{
			ptmp = pset[i];
			y = ptmp.x;
			x = ptmp.y;
			//rgb��ɫ�ռ�����Ϊ16*16*16 bins
			//rgb��ɫ�ռ�����Ϊ16*16*16 bins
			q_r = ((unsigned char)src.at<cv::Vec3b>(x, y)[2]) / 16;
			q_g = ((unsigned char)src.at<cv::Vec3b>(x, y)[1]) / 16;
			q_b = ((unsigned char)src.at<cv::Vec3b>(x, y)[0]) / 16;
			q_temp[i] = q_r * 256 + q_g * 16 + q_b;
			hist2[q_temp[i]] = hist2[q_temp[i]] + m_wei[i];
		}

		//��һ��ֱ��ͼ
		for (i = 0; i<4096; i++)
		{
			hist2[i] = hist2[i] / C;
			//printf("%f\n",hist2[i]);
		}
		//����Ŀ��ֱ��ͼ
		double temp_max = 0.0;

		for (i = 0; i<4096; i++)			//��ֱ��ͼ���ֵ��Ϊ�˹�һ��
		{
			if (temp_max < hist2[i])
			{
				temp_max = hist2[i];
			}
		}
		//��ֱ��ͼ
		cv::Point p1, p2;
		double bin_width = (double)pic_hist.cols / 4368;
		double bin_unith = (double)pic_hist.rows / temp_max;

		/*for (i = 0; i < 4096; i++)
		{
		p1.x = i * bin_width;
		p1.y = pic_hist.rows;
		p2.x = (i + 1)*bin_width;
		p2.y = pic_hist.rows - hist2[i] * bin_unith;
		rectangle(pic_hist, p1, p2, cvScalar(0, 255, 0), -1, 8, 0);
		}
		imwrite("hist2.jpg", pic_hist);*/

		for (i = 0; i < 4096; i++)
		{
			if (hist2[i] != 0)
			{
				w[i] = sqrt(hist1[i] / hist2[i]);
			}
			else
			{
				w[i] = 0;
			}
		}

		sum_w = 0.0;
		x1 = 0.0;
		x2 = 0.0;

		int tmpw, tmph;

		for (i = 0; i < count; i++)
		{
			ptmp = pset[i];
			x = ptmp.x;
			y = ptmp.y;
			tmpw = x - center.x;
			tmph = y - center.y;
			//printf("%d\n", q_temp[i * t_w + j]);
			sum_w = sum_w + w[q_temp[i]];
			//????????????????????20170531 14:51
			x1 = x1 + w[q_temp[i]] * tmph;//y
			x2 = x2 + w[q_temp[i]] * tmpw;//x
		}
		y1 = x1 / sum_w;
		y2 = x2 / sum_w;
		//if (y1 > 0.8) y1 = 1;
		//���ĵ�λ�ø���
		center.x += y2;
		center.y += y1;

		//printf("%d,%d\n",drawing_box.x,drawing_box.y);
	}
	free(hist2);
	free(w);
	free(q_temp);
	//��ʾ���ٽ��
	//ellip.angle = 0;
	ellip.center = center;

	pic_hist.release();
}

//��Բ�Ļ���
void VideoTrace::draw_ellp()
{
	int offset_x, offset_y;
	//offset_x = ellip.center.x- center_stad.x;
	//offset_y = ellip.center.y - center_stad.y;
	offset_x = drawing_box.x - local_lx;
	offset_y = drawing_box.y - local_ly;
	ellip.center.x = center_stad.x + offset_x;
	ellip.center.y = center_stad.y + offset_y;

	ellipse(frame, ellip, 255, 1);

	for (int i = 0; i < 16; i++)
	{
		LabelPixel(label_p_stad[i].x + offset_x, label_p_stad[i].y + offset_y);
	}


	int t_h = drawing_box.height;
	int t_w = drawing_box.width;
	int t_x = drawing_box.x;
	int t_y = drawing_box.y;
	//����Ŀ��Ȩֱֵ��
	/*for (int i = t_x; i < t_x + t_w; i++)
	{
	frame.at<Vec3b>(i,t_y)[2] = 255;
	frame.at<Vec3b>(i, t_y)[1] = 0;
	frame.at<Vec3b>(i, t_y)[0] = 0;
	frame.at<Vec3b>(i, t_y + t_h)[2] = 255;
	frame.at<Vec3b>(i, t_y + t_h)[1] = 0;
	frame.at<Vec3b>(i, t_y + t_h)[0] = 0;
	}
	for (int j = t_y; j < t_y + t_h; j++)
	{
	frame.at<Vec3b>(t_x,j)[2] = 255;
	frame.at<Vec3b>(t_x, j)[1] = 0;
	frame.at<Vec3b>(t_x, j)[0] = 0;
	frame.at<Vec3b>(t_x+t_w, j)[2] = 255;
	frame.at<Vec3b>(t_x+t_w, j)[1] = 0;
	frame.at<Vec3b>(t_x+t_w, j)[0] = 0;
	}*/

	for (int i = t_x; i < t_x + t_w; i++)
	{
		frame.at<cv::Vec3b>(t_y, i)[2] = 255;
		frame.at<cv::Vec3b>(t_y, i)[1] = 0;
		frame.at<cv::Vec3b>(t_y, i)[0] = 0;
		frame.at<cv::Vec3b>(t_y + t_h, i)[2] = 255;
		frame.at<cv::Vec3b>(t_y + t_h, i)[1] = 0;
		frame.at<cv::Vec3b>(t_y + t_h, i)[0] = 0;
	}
	for (int j = t_y; j < t_y + t_h; j++)
	{
		frame.at<cv::Vec3b>(j, t_x)[2] = 255;
		frame.at<cv::Vec3b>(j, t_x)[1] = 0;
		frame.at<cv::Vec3b>(j, t_x)[0] = 0;
		frame.at<cv::Vec3b>(j, t_x + t_w)[2] = 255;
		frame.at<cv::Vec3b>(j, t_x + t_w)[1] = 0;
		frame.at<cv::Vec3b>(j, t_x + t_w)[0] = 0;
	}
}

//�ֲ��������Բ�Ļ���
void VideoTrace::MyLine(cv::Point start, cv::Point end)
{
	int thickness = 2;
	int lineType = 8;
	line(frame,
		start,
		end,
		cv::Scalar(0, 255, 0),
		thickness,
		lineType);
}

void VideoTrace::label_needle(int sort)
{
	MyLine(p_n[sort], p_no[sort]);
}

void VideoTrace::draw_ellp_choose(int ith)
{
	//cout << "begin_draw" << endl;
	int offset_x, offset_y;
	int offset_x_std, offset_y_std;
	int h_max, w_max, h_min, w_min;

	offset_x = drawing_box.x - x[ith];
	offset_y = drawing_box.y - y[ith];

	/*%%%%%%%%%%%%%%%%%#######%%%%%%%%%%%%%%%%%*/
	//�����Ƿ���п������˲�
	if (abs(offset_x) > MAX_OFFSET_PER_FRAME || abs(offset_y) > MAX_OFFSET_PER_FRAME)
	{
		int offset_x_cam, offset_y_cam;
		offset_x_cam = offset_x;
		offset_y_cam = offset_y;
		//cout << "use kf" << endl;
		//���ص�����һ״̬��Ԥ��ֵ
		prediction = KF.predict();

		//offset_x = prediction.at<int>(0) - ellip.center.x;
		//offset_y = prediction.at<int>(1) - ellip.center.y;
		offset_x = (int)(KF.statePre.at<float>(0) - ellip.center.x);
		offset_y = (int)(KF.statePre.at<float>(1) - ellip.center.y);

		measurement.at<float>(0) = ellip.center.x + offset_x *0.8 + offset_x_cam *0.2;
		measurement.at<float>(1) = ellip.center.y + offset_y *0.8 + offset_y_cam *0.2;
		/*ellip.center.x = (int)KF.statePre.at<float>(0);
		ellip.center.y = (int)KF.statePre.at<float>(1);*/
		ellip.center.x = ellip.center.x + offset_x;
		ellip.center.y = ellip.center.y + offset_y;
		KF.correct(measurement);
		//offset_y = (float)prediction.at<float>(1) - ellip.center.y;
		drawing_box.x = x[ith] + offset_x;
		drawing_box.y = y[ith] + offset_y;
	}
	else
	{
		//cout << "not kf" << endl;
		prediction = KF.predict();
		ellip.center.x = ellip.center.x + offset_x;
		ellip.center.y = ellip.center.y + offset_y;
		//update KF measurement(�۲�ֵ��
		measurement.at<float>(0) = (float)ellip.center.x;
		measurement.at<float>(1) = (float)ellip.center.y;
		KF.correct(measurement);
	}
	//cout << "offset_x:" << offset_x << " offset_y:" << offset_y << endl;
	/**********2017/07/19***********************/
	//ellip.center.x = center_stad.x + offset_x;
	//ellip.center.y = center_stad.y + offset_y;

	offset_x_std = ellip.center.x - center_stad.x;
	offset_y_std = ellip.center.y - center_stad.y;
	std::cout << offset_x_std << " " << offset_y_std << std::endl;
	if (offset_x_std > OFFSETMAX || offset_y_std > OFFSETMAX || -offset_x_std > OFFSETMAX || -offset_y_std > OFFSETMAX)
	{
		ith = 0;
		reset();
	}
	else
	{
		for (int i = 0; i < 9; i++)
		{
			x[i] = x[i] + offset_x;
			y[i] = y[i] + offset_y;
		}

		ellipse(frame, ellip, 255, 1);

		for (int i = 0; i < 16; i++)
		{
			//LabelPixel(label_p_stad[i].x + offset_x, label_p_stad[i].y + offset_y);
			label_p_stad[i].x = label_p_stad[i].x + offset_x;
			label_p_stad[i].y = label_p_stad[i].y + offset_y;
			p_n[i].x = p_n[i].x + offset_x;
			p_no[i].x = p_no[i].x + offset_x;
			p_n[i].y = p_n[i].y + offset_y;
			p_no[i].y = p_no[i].y + offset_y;
			//if (label_p_stad[i].x <= 0 || label_p_stad[i].x >= r || label_p_stad[i].y <= 0 || label_p_stad[i].y >= c)
			if (p_no[i].x <= 0 || p_no[i].x >= c || p_no[i].y <= 0 || p_no[i].y >= r)
				continue;
			LabelPixel(label_p_stad[i].x, label_p_stad[i].y);
			label_needle(i);
		}

		int point_off_tm = ellip.center.x - p_n[0].x;
		if (point_off_tm != point_off)
		{
			std::cout << "error_point" << std::endl;
		}

		int t_h = drawing_box.height;
		int t_w = drawing_box.width;
		int t_x = drawing_box.x;
		int t_y = drawing_box.y;
		h_max = t_h + t_y;
		w_max = t_w + t_x;
		//cout << t_x << " " << t_y << " " << h_max << " " << w_max << endl;
		if (h_max >= r) h_max = r - 1;
		if (w_max >= c) w_max = c - 1;
		if (t_x < 0) t_x = 0;
		if (t_y < 0) t_y = 0;
		//cout << t_x << " " << t_y << " " << h_max << " " << w_max << endl;

		for (int i = t_x; i < w_max; i++)
		{
			frame.at<cv::Vec3b>(t_y, i)[2] = 255;
			frame.at<cv::Vec3b>(t_y, i)[1] = 0;
			frame.at<cv::Vec3b>(t_y, i)[0] = 0;
			frame.at<cv::Vec3b>(h_max, i)[2] = 255;
			frame.at<cv::Vec3b>(h_max, i)[1] = 0;
			frame.at<cv::Vec3b>(h_max, i)[0] = 0;
		}
		for (int j = t_y; j < h_max; j++)
		{
			frame.at<cv::Vec3b>(j, t_x)[2] = 255;
			frame.at<cv::Vec3b>(j, t_x)[1] = 0;
			frame.at<cv::Vec3b>(j, t_x)[0] = 0;
			frame.at<cv::Vec3b>(j, w_max)[2] = 255;
			frame.at<cv::Vec3b>(j, w_max)[1] = 0;
			frame.at<cv::Vec3b>(j, w_max)[0] = 0;
		}

		//cout << "drawend" << endl;
	}
	int e_x = ellip.center.y;
	int e_y = ellip.center.x;

}


//׷�ٳ�ʼ��
void VideoTrace::begin_tracing()
{
	draw_ellp();
	//��ʾ
	//imshow("�۽�Ĥ�������", frame);
	//Ŀ���ʼ��
	Init_ell_traceRect();
	hist1 = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	m_wei = (double *)malloc(sizeof(double)*drawing_box.height*drawing_box.width);
	init_target(frame);
	//@right
	//m_wei = (double *)malloc(sizeof(double)*pset.size());
	//init_target_ellpise(frame);
	//@right
	std::cout << "begin" << std::endl;
	is_tracking = true;

}

//���ھֲ�׷���ų��ڵ�����Ⱦ��
//��ʼ������λ�õ�ֱ��ͼ��3*3�еĵ�4��
void VideoTrace::begin_tracing_choose_cen()
{
	init_target_choose(frame, x[4], y[4], rt_w, rt_h, 4);
	is_tracking = true;
}

void VideoTrace::begin_tracing_choose()
{
	//draw_ellp();
	//��ʾ
	//imshow("�۽�Ĥ�������", frame);
	//Ŀ���ʼ��
	Init_ell_traceRect();
	histc1[0] = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	histc1[1] = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	histc1[2] = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	histc1[3] = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	histc1[4] = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	histc1[5] = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	histc1[6] = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	histc1[7] = (double *)malloc(sizeof(double)* 16 * 16 * 16);
	histc1[8] = (double *)malloc(sizeof(double)* 16 * 16 * 16);

	//ע�⣡������������������������
	m_wei = (double *)malloc(sizeof(double)*drawing_box.height*drawing_box.width);

	//��ʼ��3*3
	init_w();
	init_target_choose(frame, x[0], y[0], rt_w, rt_h, 0);
	init_target_choose(frame, x[1], y[1], rt_w, rt_h, 1);
	init_target_choose(frame, x[2], y[2], rt_w, rt_h, 2);
	init_target_choose(frame, x[3], y[3], rt_w, rt_h, 3);
	//init_target_choose(frame, x[4], y[4], rt_w, rt_h, 4);
	init_target_choose(frame, x[5], y[5], rt_w, rt_h, 5);
	init_target_choose(frame, x[6], y[6], rt_w, rt_h, 6);
	init_target_choose(frame, x[7], y[7], rt_w, rt_h, 7);
	init_target_choose(frame, x[8], y[8], rt_w, rt_h, 8);
	//init_target_choose(frame, x[4], y[4], rt_w, rt_h, 4);

	//is_tracking = true;

}

//ith Ϊ�ڱ�֡�б��ڵ���3*3�еĵ�ith������
int VideoTrace::choose_area(int ith)
{
	int re;
	for (int i = 0; i < 9; i++)
	{
		if (i == ith) continue;
		drawing_box.x = x[i];
		drawing_box.y = y[i];
		drawing_box.width = rt_w;
		drawing_box.height = rt_h;
		re = MeanShift_Tracking_choose(frame, i);
		if (re == -1)
			return i;

	}
	return -1;
}

/*�������˲�*/
void VideoTrace::KF_process(int offset_x, int offset_y)
{
	//kalman prediction
	prediction = KF.predict();
	//update measurement
	//�˲����ҽ�prediction��Ϊ��ʵֵȥ�����۲���
	//update
	KF.correct(prediction);


}

//��ȡ֡��ѵ��������
void VideoTrace::tracing()
{
	int re;

	for (int i = 0; i < frameNum; i++)
	{
		captureVideo >> frame;

		if (is_tracking)
		{
			//cout << i << endl;
			//@right
			//MeanShift_Tracking_ellpise(frame);
			MeanShift_Tracking(frame);
			draw_ellp();
		}

		int c = cvWaitKey(1);
		//��ͣ
		if (c == 'p')
		{
			pause = true;
			getEFCEdgeset();
			//begin_tracing();
			draw_ellp();
		}
		else if (c == 't')
		{
			begin_tracing();
		}
		if (pause == true){
			draw_ellp();
		}

		//��ʾ
		imshow("�۽�Ĥ���1����", frame);
	}
}

void VideoTrace::tracing_choose()
{
	int re, re1, ith = 0;
	//����һ֡��δ�����ȡ�ĵ㼯��Ϣ���ڽ�������ÿһ֡�����»�ȡ
	bool re_get = false;

	for (int i = 0; i < frameNum - 1; i++)
	{
		captureVideo >> frame;

		if (is_tracking)
		{
			drawing_box.x = x[ith];
			drawing_box.y = y[ith];
			drawing_box.width = rt_w;
			drawing_box.height = rt_h;
			re = MeanShift_Tracking_choose(frame, ith);
			if (re != -1)
			{
				ith = choose_area(ith);
				if (ith == -1)
				{
					ith = RESETAREA;
					//%%%%%%%%%%%%%%%%%%%%%%%%%
					MeanShift_Tracking_choose(frame, ith);
					//%%%%%%%%%%%%%%
					drawing_box.x = x[ith];
					drawing_box.y = y[ith];
					drawing_box.width = rt_w;
					drawing_box.height = rt_h;
				}
			}
			draw_ellp_choose(ith);
		}

		int c = cvWaitKey(1);
		//��ͣ
		if (c == 'p' || re_get)
		{
			pause = true;
			re1 = getEFCEdgeset();
			if (re1 == 0)
			{
				pause = false;
				re_get = true;
			}
			else
			{
				re_get = false;
				draw_ellp();
				//20170721
				begin_tracing_choose();
			}

		}
		else if (c == 't')
		{
			pause = false;
			begin_tracing_choose_cen();
			//begin_tracing_choose();
		}
		if (pause == true){
			draw_ellp();
		}

		//��ʾ
		imshow("�۽�Ĥ���1����", frame);
		//waitKey(0);
	}
}

//����׷��ÿһ��֡��c#+u3d�������ؽ��֡
//@video_index:ʹ�õ�video_group�е�����ͷindex
uchar* VideoTrace::tracing_each_frame(int video_index)
{
	int re, re1, ith = 0;
	//����һ֡��δ�����ȡ�ĵ㼯��Ϣ���ڽ�������ÿһ֡�����»�ȡ
	bool re_get = false;

	captureVideo_g[video_index] >> frame;
	if (is_tracking)
	{
		drawing_box.x = x[ith];
		drawing_box.y = y[ith];
		drawing_box.width = rt_w;
		drawing_box.height = rt_h;
		re = MeanShift_Tracking_choose(frame, ith);
		if (re != -1)
		{
			ith = choose_area(ith);
			if (ith == -1)
			{
				ith = RESETAREA;
				//%%%%%%%%%%%%%%%%%%%%%%%%%
				MeanShift_Tracking_choose(frame, ith);
				//%%%%%%%%%%%%%%
				drawing_box.x = x[ith];
				drawing_box.y = y[ith];
				drawing_box.width = rt_w;
				drawing_box.height = rt_h;
			}
		}
		draw_ellp_choose(ith);
	}

	int c = cvWaitKey(1);
	//��ͣ
	if (c == 'p' || re_get)
	{
		pause = true;
		re1 = getEFCEdgeset();
		if (re1 == 0)
		{
			pause = false;
			re_get = true;
		}
		else
		{
			re_get = false;
			draw_ellp();
			//20170721
			begin_tracing_choose();
		}

	}
	else if (c == 't')
	{
		pause = false;
		begin_tracing_choose_cen();
		//begin_tracing_choose();
	}
	if (pause == true){
		draw_ellp();
	}
	imshow("�۽�Ĥ���1����", frame);
	return frame.data;
}

//�ֲ�׷��
void VideoTrace::init_target(cv::Mat & src)
{
	IplImage *pic_hist = 0;
	int t_h, t_w, t_x, t_y;
	double h, dist;
	int i, j;
	int q_r, q_g, q_b, q_temp;

	t_h = drawing_box.height;
	t_w = drawing_box.width;
	t_x = drawing_box.x;
	t_y = drawing_box.y;

	h = pow(((double)t_w) / 2, 2) + pow(((double)t_h) / 2, 2);			//����
	pic_hist = cvCreateImage(cvSize(300, 200), IPL_DEPTH_8U, 3);     //����ֱ��ͼͼ��

	//��ʼ��Ȩֵ�����Ŀ��ֱ��ͼ
	for (i = 0; i < t_w*t_h; i++)
	{
		m_wei[i] = 0.0;
	}

	for (i = 0; i<4096; i++)
	{
		hist1[i] = 0.0;
	}

	for (i = 0; i < t_h; i++)
	{
		for (j = 0; j < t_w; j++)
		{
			dist = pow(i - (double)t_h / 2, 2) + pow(j - (double)t_w / 2, 2);
			m_wei[i * t_w + j] = 1 - dist / h;
			//printf("%f\n",m_wei[i * t_w + j]);
			C += m_wei[i * t_w + j];
		}
	}

	//����Ŀ��Ȩֱֵ��
	for (i = t_y; i < t_y + t_h; i++)
	{
		for (j = t_x; j < t_x + t_w; j++)
		{
			//rgb��ɫ�ռ�����Ϊ16*16*16 bins
			q_r = ((unsigned char)src.at<cv::Vec3b>(i, j)[2]) / 16;
			q_g = ((unsigned char)src.at<cv::Vec3b>(i, j)[1]) / 16;
			q_b = ((unsigned char)src.at<cv::Vec3b>(i, j)[0]) / 16;
			q_temp = q_r * 256 + q_g * 16 + q_b;
			hist1[q_temp] = hist1[q_temp] + m_wei[(i - t_y) * t_w + (j - t_x)];
		}
	}

	//��һ��ֱ��ͼ
	for (i = 0; i<4096; i++)
	{
		hist1[i] = hist1[i] / C;
		//printf("%f\n",hist1[i]);
	}

	//����Ŀ��ֱ��ͼ
	double temp_max = 0.0;

	for (i = 0; i < 4096; i++)			//��ֱ��ͼ���ֵ��Ϊ�˹�һ��
	{
		//printf("%f\n",val_hist[i]);
		if (temp_max < hist1[i])
		{
			temp_max = hist1[i];
		}
	}
	//��ֱ��ͼ
	CvPoint p1, p2;
	double bin_width = (double)pic_hist->width / 4096;
	double bin_unith = (double)pic_hist->height / temp_max;

	for (i = 0; i < 4096; i++)
	{
		p1.x = i * bin_width;
		p1.y = pic_hist->height;
		p2.x = (i + 1)*bin_width;
		p2.y = pic_hist->height - hist1[i] * bin_unith;
		//printf("%d,%d,%d,%d\n",p1.x,p1.y,p2.x,p2.y);
		cvRectangle(pic_hist, p1, p2, cvScalar(0, 255, 0), -1, 8, 0);
	}
	cvSaveImage("hist1.jpg", pic_hist);
	cvReleaseImage(&pic_hist);
}



void VideoTrace::MeanShift_Tracking(cv::Mat & src)
{
	int num = 0, i = 0, j = 0;
	int t_w = 0, t_h = 0, t_x = 0, t_y = 0;
	double *w = 0, *hist2 = 0;
	double sum_w = 0, x1 = 0, x2 = 0, y1 = 2.0, y2 = 2.0;
	int q_r, q_g, q_b;
	int *q_temp;
	IplImage *pic_hist = 0;

	t_w = drawing_box.width;
	t_h = drawing_box.height;

	pic_hist = cvCreateImage(cvSize(300, 200), IPL_DEPTH_8U, 3);     //����ֱ��ͼͼ��
	hist2 = (double *)malloc(sizeof(double)* 4096);
	w = (double *)malloc(sizeof(double)* 4096);
	q_temp = (int *)malloc(sizeof(int)*t_w*t_h);

	while ((pow(y2, 2) + pow(y1, 2) > 0.5) && (num < NUM))
	{
		num++;
		t_x = drawing_box.x;
		t_y = drawing_box.y;
		memset(q_temp, 0, sizeof(int)*t_w*t_h);
		for (i = 0; i<4096; i++)
		{
			w[i] = 0.0;
			hist2[i] = 0.0;
		}

		for (i = t_y; i < t_h + t_y; i++)
		{
			for (j = t_x; j < t_w + t_x; j++)
			{
				//rgb��ɫ�ռ�����Ϊ16*16*16 bins
				//rgb��ɫ�ռ�����Ϊ16*16*16 bins
				q_r = ((unsigned char)src.at<cv::Vec3b>(i, j)[2]) / 16;
				q_g = ((unsigned char)src.at<cv::Vec3b>(i, j)[1]) / 16;
				q_b = ((unsigned char)src.at<cv::Vec3b>(i, j)[0]) / 16;
				q_temp[(i - t_y) *t_w + j - t_x] = q_r * 256 + q_g * 16 + q_b;
				hist2[q_temp[(i - t_y) *t_w + j - t_x]] = hist2[q_temp[(i - t_y) *t_w + j - t_x]] + m_wei[(i - t_y) * t_w + j - t_x];
			}
		}

		//��һ��ֱ��ͼ
		for (i = 0; i<4096; i++)
		{
			hist2[i] = hist2[i] / C;
			//printf("%f\n",hist2[i]);
		}
		//����Ŀ��ֱ��ͼ
		double temp_max = 0.0;

		for (i = 0; i<4096; i++)			//��ֱ��ͼ���ֵ��Ϊ�˹�һ��
		{
			if (temp_max < hist2[i])
			{
				temp_max = hist2[i];
			}
		}


		for (i = 0; i < 4096; i++)
		{
			if (hist2[i] != 0)
			{
				w[i] = sqrt(hist1[i] / hist2[i]);
			}
			else
			{
				w[i] = 0;
			}
		}

		sum_w = 0.0;
		x1 = 0.0;
		x2 = 0.0;

		for (i = 0; i < t_h; i++)
		{
			for (j = 0; j < t_w; j++)
			{
				//printf("%d\n",q_temp[i * t_w + j]);
				sum_w = sum_w + w[q_temp[i * t_w + j]];
				x1 = x1 + w[q_temp[i * t_w + j]] * (i - t_h / 2);
				x2 = x2 + w[q_temp[i * t_w + j]] * (j - t_w / 2);
			}
		}
		y1 = x1 / sum_w;
		y2 = x2 / sum_w;
		//cout << y1 << " " << y2 << endl;
		if (y1>-0.6 && y1 <0) y1 = 0;
		if (y2>-0.6 && y2 <0) y2 = 0;
		//���ĵ�λ�ø���
		drawing_box.x += y2;
		drawing_box.y += y1;
	}
	//cout << num << endl;
	free(hist2);
	free(w);
	free(q_temp);
	cvReleaseImage(&pic_hist);
}

//��ǰ��ʼ��Ȩ��
void VideoTrace::init_w()
{
	int t_h, t_w;
	int i, j;
	double dist, h;

	t_h = drawing_box.height;
	t_w = drawing_box.width;

	h = pow(((double)t_w) / 2, 2) + pow(((double)t_h) / 2, 2);			//����

	//��ʼ��Ȩֵ�����Ŀ��ֱ��ͼ
	for (i = 0; i < t_w*t_h; i++)
	{
		m_wei[i] = 0.0;
	}

	for (i = 0; i < t_h; i++)
	{
		for (j = 0; j < t_w; j++)
		{
			dist = pow(i - (double)t_h / 2, 2) + pow(j - (double)t_w / 2, 2);
			m_wei[i * t_w + j] = 1 - dist / h;
			//printf("%f\n",m_wei[i * t_w + j]);
			C += m_wei[i * t_w + j];
		}
	}
}

void VideoTrace::init_target_choose(cv::Mat & src, int ori_x, int ori_y, int ori_w, int ori_h, int ith)
{
	int t_h, t_w, t_x, t_y;
	double h, dist;
	int i, j;
	int q_r, q_g, q_b, q_temp;
	int h_max, w_max;

	t_h = ori_h;
	t_w = ori_w;
	t_x = ori_x;
	t_y = ori_y;

	h = pow(((double)t_w) / 2, 2) + pow(((double)t_h) / 2, 2);			//����

	//cout << "init_target_choose" << ith << endl;

	for (i = 0; i<4096; i++)
	{
		histc1[ith][i] = 0.0;
	}

	//cout << "begin:1" << endl;
	h_max = t_h + t_y;
	w_max = t_w + t_x;
	if (h_max >= r) h_max = r - 1;
	if (w_max >= c) w_max = c - 1;
	if (t_x < 0) t_x = 0;
	if (t_y < 0) t_y = 0;
	//����Ŀ��Ȩֱֵ��
	for (i = t_y; i < h_max; i++)
	{
		for (j = t_x; j < w_max; j++)
		{
			//rgb��ɫ�ռ�����Ϊ16*16*16 bins
			q_r = ((unsigned char)src.at<cv::Vec3b>(i, j)[2]) / 16;
			q_g = ((unsigned char)src.at<cv::Vec3b>(i, j)[1]) / 16;
			q_b = ((unsigned char)src.at<cv::Vec3b>(i, j)[0]) / 16;
			q_temp = q_r * 256 + q_g * 16 + q_b;
			histc1[ith][q_temp] = histc1[ith][q_temp] + m_wei[(i - t_y) * t_w + (j - t_x)];
		}
	}
	//cout << "begin:1" << endl;

	//��һ��ֱ��ͼ
	for (i = 0; i<4096; i++)
	{
		histc1[ith][i] = histc1[ith][i] / C;
		//printf("%f\n",hist1[i]);
	}
	//cout << "begin:2" << endl;
	//����Ŀ��ֱ��ͼ
	double temp_max = 0.0;

	for (i = 0; i < 4096; i++)			//��ֱ��ͼ���ֵ��Ϊ�˹�һ��
	{
		//printf("%f\n",val_hist[i]);
		if (temp_max < histc1[ith][i])
		{
			temp_max = histc1[ith][i];
		}
	}
}


int VideoTrace::MeanShift_Tracking_choose(cv::Mat & src, int ith)
{
	int num = 0, i = 0, j = 0;
	int t_w = 0, t_h = 0, t_x = 0, t_y = 0;
	double *w = 0, *hist2 = 0;
	double sum_w = 0, x1 = 0, x2 = 0, y1 = 2.0, y2 = 2.0;
	int q_r, q_g, q_b;
	int *q_temp;
	int h_max, w_max;

	t_w = rt_w;
	t_h = rt_h;

	hist2 = (double *)malloc(sizeof(double)* 4096);
	w = (double *)malloc(sizeof(double)* 4096);
	q_temp = (int *)malloc(sizeof(int)*t_w*t_h);

	//cout << "MeanShift_Tracking_choose:" << ith << endl;

	while ((pow(y2, 2) + pow(y1, 2) > 0.5) && (num < NUM))
	{
		num++;
		t_x = drawing_box.x;
		t_y = drawing_box.y;
		memset(q_temp, 0, sizeof(int)*t_w*t_h);
		//cout << "init" << endl;
		for (i = 0; i<4096; i++)
		{
			w[i] = 0.0;
			hist2[i] = 0.0;
		}
		//cout << "init1" << endl;
		h_max = t_h + t_y;
		w_max = t_w + t_x;
		if (h_max >= r) h_max = r - 1;
		if (w_max >= c) w_max = c - 1;
		if (t_x < 0) t_x = 0;
		if (t_y < 0) t_y = 0;
		//cout << t_y << " " << t_x << " " << h_max << " " << w_max << endl;
		for (i = t_y; i < h_max; i++)
		{
			for (j = t_x; j < w_max; j++)
			{
				//rgb��ɫ�ռ�����Ϊ16*16*16 bins
				//rgb��ɫ�ռ�����Ϊ16*16*16 bins
				q_r = ((unsigned char)src.at<cv::Vec3b>(i, j)[2]) / 16;
				q_g = ((unsigned char)src.at<cv::Vec3b>(i, j)[1]) / 16;
				q_b = ((unsigned char)src.at<cv::Vec3b>(i, j)[0]) / 16;
				q_temp[(i - t_y) *t_w + j - t_x] = q_r * 256 + q_g * 16 + q_b;
				hist2[q_temp[(i - t_y) *t_w + j - t_x]] = hist2[q_temp[(i - t_y) *t_w + j - t_x]] + m_wei[(i - t_y) * t_w + j - t_x];
			}
		}
		//��һ��ֱ��ͼ
		for (i = 0; i<4096; i++)
		{
			hist2[i] = hist2[i] / C;
			//printf("%f\n",hist2[i]);
		}
		//����Ŀ��ֱ��ͼ
		double temp_max = 0.0;

		for (i = 0; i<4096; i++)			//��ֱ��ͼ���ֵ��Ϊ�˹�һ��
		{
			if (temp_max < hist2[i])
			{
				temp_max = hist2[i];
			}
		}

		for (i = 0; i < 4096; i++)
		{
			if (hist2[i] != 0)
			{
				w[i] = sqrt(histc1[ith][i] / hist2[i]);
			}
			else
			{
				w[i] = 0;
			}
		}

		sum_w = 0.0;
		x1 = 0.0;
		x2 = 0.0;
		for (i = 0; i < t_h; i++)
		{
			for (j = 0; j < t_w; j++)
			{
				//printf("%d\n",q_temp[i * t_w + j]);
				sum_w = sum_w + w[q_temp[i * t_w + j]];
				x1 = x1 + w[q_temp[i * t_w + j]] * (i - t_h / 2);
				x2 = x2 + w[q_temp[i * t_w + j]] * (j - t_w / 2);
			}
		}
		//cout << "sum_w" << sum_w << endl;
		if (sum_w != 0)
		{
			y1 = x1 / sum_w;
			y2 = x2 / sum_w;
			//cout << y1 << " " << y2 << endl;
		}
		else
		{
			y1 = y2 = 0;
		}

		//�ж��Ƿ��ڵ�����Ⱦ�Ľ���
		if (y1 > IFSHELETER || -y1 > IFSHELETER || y2 > IFSHELETER || -y2 > IFSHELETER)
		{
			free(hist2);
			free(w);
			free(q_temp);
			//�������ڵ����ⲿ�ֵ����
			//cout << "return" << endl;
			return ith;
		}
		/*if (y1 > 0)
		{
		y1 = y1 + 0.5;
		}
		else if (y1 > -0.5 && y1 <0)
		{
		y1 = 0;
		}
		if (y2 > 0)
		{
		y2 = y2 + 0.5;
		}
		else if (y2 > -0.5 && y2 <0)
		{
		y2 = 0;
		}*/
		y1 = y1 + 0.5;
		y2 = y2 + 0.5;
		//���ĵ�λ�ø���
		drawing_box.x += y2;
		drawing_box.y += y1;
	}
	//cout << num << endl;
	free(hist2);
	free(w);
	free(q_temp);

	return -1;
}

