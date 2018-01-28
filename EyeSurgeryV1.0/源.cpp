#include<iostream>
#include<fstream>
#include<vector>
#include<cstdlib>
#include "FindEll.h"
#include "EFCHandler.h"
#include "VideoTrace.h"
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	/*VideoTrace vt("9.mp4", "out.txt");
	vt.tracing_choose();*/
	VideoTrace vt(4);
	while (1)
	{
		vt.tracing_each_frame(0);
	}
	cv::waitKey(0);
}
