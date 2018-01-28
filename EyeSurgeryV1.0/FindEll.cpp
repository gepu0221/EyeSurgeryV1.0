//#include "edopencv.h"
#include "FindEll.h"
#include "EFCHandler.h"


/*
//�������еĵ�ת��Ϊ��Mat��ʽ�洢��2ά�㼯(x,y)
Mat(contours[i]).convertTo(pointsf, CV_32F);
*/
//�ж��Ƿ���߽�
int FindEll::JudgeOutEdge(int c_x, int c_y, int w, int h, int rr, int cc)
{
	int l_x = c_x - w / 2;
	int l_y = c_y - h / 2;
	int r_x = l_x + w;
	int r_y = l_y + h;
	if (l_x > 0 && l_y > 0 && r_x < cc&&r_y < rr)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//��Canny��Ե�㷨��ȡ��Ե
void FindEll::CannyEdge()
{
	cvtColor(src, dst, CV_RGB2GRAY);
	GaussianBlur(dst, dst, cv::Size(3, 3), 0, 0);       //��˹ģ����Gaussian Blur��
	Canny(dst, dst, lowThreshold, maxThreshold, 3);
	/*dst = edo.SeedFill(dst);//Canny��Ե���
	edopencv edo1;
	edo1.Init(src.cols, src.rows);
	dst = edo1.SeedFill(dst);*/
	imwrite("canny.bmp", dst);
}

//�����޸Ĵ�С��Canny��Ե�㷨��ȡ��Ե
void FindEll::ResizeCannyEdge()
{
	cv::Mat re_dst = cv::Mat(r*resize_rate, c*resize_rate, CV_8UC3, cv::Scalar(0));
	cvtColor(src, dst, CV_RGB2GRAY);
	GaussianBlur(dst, dst, cv::Size(3, 3), 0, 0);       //��˹ģ����Gaussian Blur��
	imwrite("canny0.bmp", dst);
	cv::resize(dst, re_dst, cv::Size(0, 0), resize_rate, resize_rate);
	imwrite("canny1.bmp", dst);
	Canny(re_dst, re_dst, lowThreshold, maxThreshold, 3);
	cv::resize(re_dst, dst, cv::Size(0, 0), 1 / resize_rate, 1 / resize_rate);
	imwrite("canny_re.bmp", re_dst);
	imwrite("canny.bmp", dst);
}

void FindEll::EllipseEx_my(cv::Mat& img, cv::Point center, cv::Size axes,
	int angle, int arc_start, int arc_end, std::vector<cv::Point> & v)
{
	axes.width = std::abs(axes.width), axes.height = std::abs(axes.height);
	int delta = (std::max(axes.width, axes.height) + (XY_ONE >> 1)) >> XY_SHIFT;
	delta = 1;
	ellipse2Poly(center, axes, angle, arc_start, arc_end, delta, v);
}

void FindEll::ellipse_my(cv::Mat& img, const cv::RotatedRect& box, std::vector<cv::Point> &v)
{

	CV_Assert(box.size.width >= 0 && box.size.height >= 0);

	int _angle = cvRound(box.angle);
	//int _angle = cvRound(0);
	cv::Point center(cvRound(box.center.x), cvRound(box.center.y));
	cv::Size axes(cvRound(box.size.width / 2), cvRound(box.size.height / 2));
	EllipseEx_my(img, center, axes, _angle, 0, 360, v);
}


//Ѱ�ұ߽磨�����Բ��
int FindEll::findContour()
{
	contours.clear();
	findContours(dst, contours, hierarchy, cv::RETR_TREE, CV_CHAIN_APPROX_NONE);

	/*�ҵ��������������е�ĸ����������Σ�����ϲ�*/
	int max, secmax = -1, lmax, lsmax;
	std::vector<cv::Point> t1;//�ݴ�ζ���ĵ㼯
	max = contours[0].size();
	lmax = 0;
	for (int i = 1; i < contours.size(); i++)
	{
		oripset = contours[i];
		if (oripset.size()>max)
		{
			secmax = max;
			max = oripset.size();
			lsmax = lmax;
			lmax = i;
		}
	}

	//�ϲ�����
	//imshow("�����ƽ������ͼ��", src);
	oripset = contours[lmax];
	if (secmax != -1)
	{
		t1 = contours[lsmax];
		for (int i = 0; i < t1.size(); i++)
		{
			oripset.push_back(t1[i]);
		}
	}





	for (int i = 0; i < show.rows; i++)
	{
		for (int j = 0; j < show.cols; j++)
		{
			show.at<uchar>(i, j) = 0;
		}
	}
	/**********show*******/

	ellipsemege = fitEllipse(oripset);
	int re = JudgeOutEdge(ellipsemege.center.x, ellipsemege.center.y, ellipsemege.size.width, ellipsemege.size.height, r, c);
	if (re == 0)
	{
		return re;
	}
	/**********show*******/
	//���ݵõ�����Բ����Ϣ  ������Բ
	ellipse(show, ellipsemege, 255, 1);
	//imshow("�����ƽ������ͼ��", show);
	/**********show*******/
	return re;
}

//Ѱ�ұ߽磨�����Բ������õ㼯
int FindEll::findContourGetPset()
{
	contours.clear();
	//imwrite("label10.bmp", dst);
	//findContours(dst, contours, hierarchy, cv::RETR_TREE, CV_CHAIN_APPROX_NONE);
	findContours(dst, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//findContours(dst, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	/*�ҵ��������������е�ĸ����������Σ�����ϲ�*/
	int max, secmax = -1, lmax, lsmax;
	std::vector<cv::Point> t1;//�ݴ�ζ���ĵ㼯
	max = contours[0].size();
	lmax = 0;
	for (int i = 1; i < contours.size(); i++)
	{
		oripset = contours[i];
		if (oripset.size()>max)
		{
			secmax = max;
			max = oripset.size();
			lsmax = lmax;
			lmax = i;
		}
	}

	//�ϲ�����
	//imshow("�����ƽ������ͼ��", src);
	oripset = contours[lmax];


	/**********show*******/

	ellipsemege = fitEllipse(oripset);

	int re = JudgeOutEdge(ellipsemege.center.x, ellipsemege.center.y, ellipsemege.size.width, ellipsemege.size.height, r, c);
	if (re == 0)
	{
		return re;
	}
	/**********show*******/
	//���ݵõ�����Բ����Ϣ  ������Բ
	//ellipsemege.angle = 0;
	ellipse_my(show, ellipsemege, dstpset);
	ellipse(show, ellipsemege, 255, 1);
	//���ݵõ�����Բ����Ϣ  ������Բ

	return re;
}


//�����������ҵı߽�
void FindEll::findtlbr()
{
	pedge[1].x = pedge[0].y = 10000;
	pedge[3].x = pedge[2].y = 0;
	int tx, ty;
	int h_w = ellipsemege.size.width / 2;
	int h_h = ellipsemege.size.height / 2;
	int tmpx, tmpy;

	pedge[1].x = ellipsemege.center.x - h_w;
	pedge[1].y = ellipsemege.center.y;
	pedge[3].x = ellipsemege.center.x + h_w;
	pedge[3].y = ellipsemege.center.y;
	pedge[0].x = ellipsemege.center.x;
	pedge[0].y = ellipsemege.center.y - h_h;
	pedge[2].x = ellipsemege.center.x;
	pedge[2].y = ellipsemege.center.y + h_h;

	for (int i = 0; i < elledge.size(); i++)
	{
		tx = elledge[i].x;
		ty = elledge[i].y;
		if (ty == pedge[1].y&&tx<ellipsemege.center.x){ numsort[1] = i; }
		if (ty == pedge[3].y&&tx>ellipsemege.center.x){ numsort[3] = i; }
		if (tx == pedge[0].x&&ty<ellipsemege.center.y){ numsort[0] = i; }
		if (tx == pedge[2].x&&ty>ellipsemege.center.y){ numsort[2] = i; }
	}

	part_label.push_back(pedge[1]);
	part_label.push_back(pedge[3]);
	part_label.push_back(pedge[0]);
	part_label.push_back(pedge[2]);

}

void FindEll::getEllSet()
{
	cv::Point ptmp;
	findContours(show, contours, hierarchy, cv::RETR_TREE, CV_CHAIN_APPROX_NONE);
	elledge = contours[0];
	findtlbr();
}

//��ԭͼ�����Բ�Ͷ���
void FindEll::LabelSrc()
{
	cv::Point ptmp;
	//��ԭͼ�ϱ����ϳ�������Բ����
	for (int i = 0; i < elledge.size(); i++)
	{
		ptmp = elledge[i];
		src.at<cv::Vec3b>(ptmp.y, ptmp.x)[0] = 0;
		src.at<cv::Vec3b>(ptmp.y, ptmp.x)[1] = 255;
		src.at<cv::Vec3b>(ptmp.y, ptmp.x)[2] = 0;

	}
	//����ĸ��ȷֵ�
	for (int i = 0; i < 4; i++)
	{
		ptmp = pedge[i];
		src.at<cv::Vec3b>(ptmp.y, ptmp.x)[0] = 0;
		src.at<cv::Vec3b>(ptmp.y, ptmp.x)[1] = 0;
		src.at<cv::Vec3b>(ptmp.y, ptmp.x)[2] = 255;

	}
}

//ͳ��ÿ���ֵ�ĸ����������Ĳ��֣�
void FindEll::caculatePoint()
{
	all = elledge.size() - 1;
	//num��ÿ�ݵĵ�ĸ���
	int tt = numsort[0];
	int c = 0, nc = 1;
	for (int i = 0; i <= all; i++)
	{
		//newpet.push_back(elledge[tt]);
		nc++;
		if (tt == numsort[c + 1])
		{
			num[c] = nc;
			nc = 1;
			c++;
		}
		if (tt != all)
		{
			tt++;
		}
		else
		{
			tt = 0;
		}
	}
	num[3] = nc;
}

//�ҵ��ԳƵ㣨y������ͬ��x���겻ͬ��
void FindEll::FindSymm(cv::Point pf, cv::Point & pr)
{
	int tx, ty;
	int fx = pf.x, fy = pf.y;
	int offset;
	int total = elledge.size();
	int i;
	for (i = 0; i < total; i++)
	{
		tx = elledge[i].x;
		ty = elledge[i].y;
		offset = abs(tx - fx);
		if (ty == fy&&offset>20)
		{
			pr.x = tx;
			pr.y = ty;
			break;
		}
	}
}

//��ǵ������ص㣨��ԭͼsrc�ϣ�
void FindEll::LabelPixel(int x, int y)
{
	src.at<cv::Vec3b>(y, x)[0] = src.at<cv::Vec3b>(y - 1, x)[0] = src.at<cv::Vec3b>(y + 1, x)[0] = src.at<cv::Vec3b>(y, x + 1)[0] = src.at<cv::Vec3b>(y, x - 1)[0] = 255;
	src.at<cv::Vec3b>(y - 1, x - 1)[0] = src.at<cv::Vec3b>(y + 1, x - 1)[0] = src.at<cv::Vec3b>(y - 1, x - 1)[0] = src.at<cv::Vec3b>(y + 1, x + 1)[0] = 255;
	src.at<cv::Vec3b>(y, x)[1] = src.at<cv::Vec3b>(y - 1, x)[1] = src.at<cv::Vec3b>(y + 1, x)[1] = src.at<cv::Vec3b>(y, x + 1)[1] = src.at<cv::Vec3b>(y, x - 1)[1] = 0;
	src.at<cv::Vec3b>(y - 1, x - 1)[1] = src.at<cv::Vec3b>(y + 1, x - 1)[1] = src.at<cv::Vec3b>(y - 1, x - 1)[1] = src.at<cv::Vec3b>(y + 1, x + 1)[1] = 0;
	src.at<cv::Vec3b>(y, x)[2] = src.at<cv::Vec3b>(y - 1, x)[2] = src.at<cv::Vec3b>(y + 1, x)[2] = src.at<cv::Vec3b>(y, x + 1)[2] = src.at<cv::Vec3b>(y, x - 1)[2] = 0;
	src.at<cv::Vec3b>(y - 1, x - 1)[2] = src.at<cv::Vec3b>(y + 1, x - 1)[2] = src.at<cv::Vec3b>(y + 1, x - 1)[2] = src.at<cv::Vec3b>(y + 1, x + 1)[2] = 0;
}

//���ĵȷֺ��һ�ݼ����ȷ��ķݣ��ȷֵ�k�ݣ�
//ֻ�����ߣ��ұ�ֱ�ӶԳƹ�ȥ
void FindEll::LabelOnePart(int k)
{
	int part = num[k] / 4;
	int count = 0, ccount = 0;
	int i;
	cv::Point ptmp, psymm;

	if (numsort[k] < numsort[k + 1])
	{
		for (i = numsort[k] + 1; i <= numsort[k + 1]; i++)
		{
			ptmp = elledge[i];
			//if (count == 0 || ccount == part)
			if (ccount == part)
			{
				int x = ptmp.x;
				int y = ptmp.y;
				LabelPixel(x, y);
				FindSymm(ptmp, psymm);
				LabelPixel(psymm.x, psymm.y);
				part_label.push_back(ptmp);
				part_label.push_back(psymm);
				count++;
				ccount = 0;
			}
			else if (count == 4)
				break;
			else
			{
				ccount++;
			}
		}
	}

	else
	{
		for (i = numsort[k] + 1; i<elledge.size(); i++)
		{
			ptmp = elledge[i];
			//if (count == 0 || ccount == part)
			if (ccount == part)
			{
				int x = ptmp.x;
				int y = ptmp.y;
				LabelPixel(x, y);
				FindSymm(ptmp, psymm);
				LabelPixel(psymm.x, psymm.y);
				part_label.push_back(ptmp);
				part_label.push_back(psymm);
				count++;
				ccount = 0;
			}
			else if (count == 4)
				break;
			else
			{
				ccount++;
			}
		}
	}
	for (i = 0; i<numsort[k + 1]; i++)
	{
		ptmp = elledge[i];
		//if (count == 0 || ccount == part)
		if (ccount == part)
		{
			int x = ptmp.x;
			int y = ptmp.y;
			LabelPixel(x, y);
			FindSymm(ptmp, psymm);
			LabelPixel(psymm.x, psymm.y);
			part_label.push_back(ptmp);
			part_label.push_back(psymm);
			count++;
			ccount = 0;
		}
		else if (count == 4)
			break;
		else
		{
			ccount++;
		}
	}
}


//��ȡ��Բ����������ͳ�����
void FindEll::getEllMesg(cv::Point & center, int & angle, int & t_w, int & t_h)
{
	center = ellipsemege.center;
	angle = ellipsemege.angle;
	t_w = ellipsemege.size.width;
	t_h = ellipsemege.size.height;
}



//SHOW
void FindEll::showResult()
{
	//imshow("��������ͼ��", src);
	imwrite("label13.bmp", src);
}

//Process
int FindEll::AllProcess()
{
	int re = 0;
	CannyEdge();
	//re = findContour();
	findContourGetPset();
	if (re == 0)
	{
		return re;
	}
	else
	{
		getEllSet();
		LabelSrc();
		caculatePoint();
		LabelOnePart(0);
		LabelOnePart(1);
		showResult();
		return re;
	}

}

int FindEll::ECFAllProcess()
{
	int re = 0;
	ResizeCannyEdge();
	re = findContourGetPset();
	if (re == 0)
	{
		return re;
	}
	else
	{
		EFCHandler efc;
		efc.Part(dstpset, part_label, coef_num, part_num);
		return re;
	}

}

//��һ��ͼ��ȫ����Ϊ0Ϊ����ʾ
void createBack(cv::Mat & show)
{
	for (int i = 0; i < show.rows; i++)
	{
		for (int j = 0; j < show.cols; j++)
		{
			show.at<uchar>(i, j) = 0;
		}
	}
}

//��������Ѱ�ұ߽�
void FindEll::resetBoundary()
{
	for (int i = 0; i < contours.size(); i++)
	{
		contours.at(i).clear();
	}
	hierarchy.clear();
	oripset.clear();
	elledge.clear();
}

