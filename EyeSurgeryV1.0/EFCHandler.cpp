#include "EFCHandler.h"

const double PI = 3.141592653589793;

std::vector<double> EFCHandler::assist_2npi;
std::vector<double> EFCHandler::assist_2_nsq_pisq;

EFCHandler::EFCHandler()
{

}


void EFCHandler::CalculateAssitVaraible(unsigned int assist_array_length)
{
	if (assist_2npi.size() < assist_array_length)
	{
		unsigned int n = assist_2npi.size();
		assist_2npi.resize(assist_array_length);
		while (n < assist_array_length)
		{
			assist_2npi[n] = 2 * n * PI;
			++n;
		}
		n = assist_2_nsq_pisq.size();
		assist_2_nsq_pisq.resize(assist_array_length);
		while (n < assist_array_length)
		{
			assist_2_nsq_pisq[n] = 2 * n * n * PI * PI;
			++n;
		}
	}
}

void EFCHandler::Part(const std::vector<cv::Point>& input_points, std::vector<cv::Point>& output_points, int reconstruction_coef_num, int part_num)
{
	double period = 1;
	std::vector<EFC> coef = PartEncode(input_points, reconstruction_coef_num, period);
	output_points = PartDecode(part_num, coef, reconstruction_coef_num);
}

std::vector<EFC> EFCHandler::PartEncode(const std::vector<cv::Point> &contour_points, unsigned int coef_num, double &_period)
{
	std::vector<EFC> coef(coef_num);
	coef[0].B = coef[0].D = 0.0;
	// �쳣���
	if (0 == contour_points.size())
	{
		return coef;
	}
	// ���㸨������
	CalculateAssitVaraible(coef_num);
	// ����deltaX, deltaY, deltaT
	unsigned int point_num = contour_points.size();
	unsigned int seg_num = point_num - 1;
	std::vector<double> deltaX(seg_num, 0.0);
	std::vector<double> deltaY(seg_num, 0.0);
	std::vector<double> deltaT(seg_num, 0.0);
	for (unsigned int i = 0; i < seg_num; ++i)
	{
		deltaX[i] = contour_points[i + 1].x - contour_points[i].x;
		deltaY[i] = contour_points[i + 1].y - contour_points[i].y;
		deltaT[i] = sqrt(deltaX[i] * deltaX[i] + deltaY[i] * deltaY[i]);
	}
	// ȥ���ظ���
	for (unsigned int i = 0; i < deltaT.size(); ++i)
	{
		if (deltaT[i] < DBL_MIN)
		{
			deltaX.erase(deltaX.cbegin() + i);
			deltaY.erase(deltaY.cbegin() + i);
			deltaT.erase(deltaT.cbegin() + i);
		}
	}
	seg_num = deltaT.size();
	point_num = seg_num + 1;
	// ����time_stamp
	std::vector<double> time_stamp(point_num, 0.0);
	time_stamp[0] = 0.0;
	for (unsigned int i = 1; i < point_num; ++i)
	{
		time_stamp[i] = time_stamp[i - 1] + deltaT[i - 1];
	}
	double period = time_stamp.back();
	//gp
	_period = period;
	// ����A0
	double ellipse_dcX = 0.0;
	for (unsigned int i = 0; i < seg_num; ++i)
	{
		double epsilon = 0.0;
		for (unsigned int j = 0; j < i; ++j)
		{
			epsilon += deltaX[j];
		}
		epsilon -= deltaX[i] / deltaT[i] * time_stamp[i];
		double single_item = deltaX[i] / (2.0 * deltaT[i]) * (time_stamp[i + 1] * time_stamp[i + 1] - time_stamp[i] * time_stamp[i]) + epsilon * (time_stamp[i + 1] - time_stamp[i]);
		ellipse_dcX += single_item;
	}
	coef[0].A = ellipse_dcX / period + contour_points.front().x;
	// ����C0
	double ellipse_dcY = 0.0;
	for (unsigned int i = 0; i < seg_num; ++i)
	{
		double epsilon = 0.0;
		for (unsigned int j = 0; j < i; ++j)
		{
			epsilon += deltaY[j];
		}
		epsilon -= deltaY[i] / deltaT[i] * time_stamp[i];
		double single_item = deltaY[i] / (2.0 * deltaT[i]) * (time_stamp[i + 1] * time_stamp[i + 1] - time_stamp[i] * time_stamp[i]) + epsilon * (time_stamp[i + 1] - time_stamp[i]);
		ellipse_dcY += single_item;
	}
	coef[0].C = ellipse_dcY / period + contour_points.front().y;
	// ����a_n
	for (unsigned int i = 1; i < coef_num; ++i)
	{
		double accum = 0.0;
		for (unsigned int j = 0; j < seg_num; ++j)
		{
			accum += (deltaX[j] / deltaT[j] * (cos(assist_2npi[i] * time_stamp[j + 1] / period) - cos(assist_2npi[i] * time_stamp[j] / period)));
		}
		coef[i].A = accum * period / assist_2_nsq_pisq[i];
	}
	// ����b_n
	for (unsigned int i = 1; i < coef_num; ++i)
	{
		double accum = 0.0;
		for (unsigned int j = 0; j < seg_num; ++j)
		{
			accum += (deltaX[j] / deltaT[j] * (sin(assist_2npi[i] * time_stamp[j + 1] / period) - sin(assist_2npi[i] * time_stamp[j] / period)));
		}
		coef[i].B = accum * period / assist_2_nsq_pisq[i];
	}
	// ����c_n
	for (unsigned int i = 1; i < coef_num; ++i)
	{
		double accum = 0.0;
		for (unsigned int j = 0; j < seg_num; ++j)
		{
			accum += (deltaY[j] / deltaT[j] * (cos(assist_2npi[i] * time_stamp[j + 1] / period) - cos(assist_2npi[i] * time_stamp[j] / period)));
		}
		coef[i].C = accum * period / assist_2_nsq_pisq[i];
	}
	// ����d_n
	for (unsigned int i = 1; i < coef_num; ++i)
	{
		double accum = 0.0;
		for (unsigned int j = 0; j < seg_num; ++j)
		{
			accum += (deltaY[j] / deltaT[j] * (sin(assist_2npi[i] * time_stamp[j + 1] / period) - sin(assist_2npi[i] * time_stamp[j] / period)));
		}
		coef[i].D = accum * period / assist_2_nsq_pisq[i];
	}

	return coef;
}


std::vector<cv::Point> EFCHandler::PartDecode(unsigned int part_num, std::vector<EFC>& coefficients, unsigned int reconstruct_coef_num)
{

	std::vector<cv::Point> contour(part_num);
	if (reconstruct_coef_num > coefficients.size())
	{
		return contour;
	}
	// ���㸨������
	CalculateAssitVaraible(reconstruct_coef_num);
	// �ؽ�
	for (unsigned int i = 0; i < part_num; ++i)
	{
		double accumX = 0.0;
		double accumY = 0.0;
		for (unsigned int j = 1; j < reconstruct_coef_num; ++j)
		{
			accumX += coefficients[j].A * cos(assist_2npi[j] * i / part_num) + coefficients[j].B * sin(assist_2npi[j] * i / part_num);
			accumY += coefficients[j].C * cos(assist_2npi[j] * i / part_num) + coefficients[j].D * sin(assist_2npi[j] * i / part_num);
		}
		contour[i].x = (float)(coefficients[0].A + accumX);
		contour[i].y = (float)(coefficients[0].C + accumY);
	}

	return contour;
}