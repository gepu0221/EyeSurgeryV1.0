#ifndef EFCHANDLER_H
#define EFCHANDLER_H

#include<iostream>
#include<cv.h>
#include<vector>
#include<opencv2/opencv.hpp>

struct EFC
{
	double A;
	double B;
	double C;
	double D;
};


// Elliptical Fourier Coefficient Handler
class EFCHandler
{
public:
	EFCHandler();

	/*------gp------------*/
	void Part(const std::vector<cv::Point>& input_points, std::vector<cv::Point>& output_points, int reconstruction_coef_num, int part_num);

	std::vector<EFC> PartEncode(const std::vector<cv::Point> &contour_points, unsigned int coef_num, double &_period);

	std::vector<cv::Point> PartDecode(unsigned int part_num, std::vector<EFC>& coefficients, unsigned int reconstruct_coef_num);
	/*--------gp---------------*/

	void CalculateAssitVaraible(unsigned int assist_array_length);
	static std::vector<double> assist_2npi;
	static std::vector<double> assist_2_nsq_pisq;
};


#endif