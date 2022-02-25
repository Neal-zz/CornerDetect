#include "Detector.h"

#include <Eigen\Dense>
#include <opencv2\opencv.hpp>

/*timer*/
auto tic = []()
{
	return std::chrono::system_clock::now();
};

auto toc = [](std::chrono::system_clock::time_point start, const std::string& name)
{
	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
	std::chrono::duration<double, std::milli> duration_ms = end - start;
	std::cout << name << '\t' << duration_ms.count() << "ms" << std::endl;

	return;
};

void testCornerDetect() {
	auto t1 = tic();

	cv::Mat img = cv::imread("markerFar.bmp");  // CV_8UC3
	if (!img.data)
	{
		printf(" No image data \n ");
		return;
	}
	cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // CV_8UC1
	img.convertTo(img, CV_32FC1, 1.0 / 255.0); // CV_32FC1

	Detector detector(img.size());
	QRsTemplate QRs = detector.process(img);

	toc(t1, "t1");

	cv::Mat imgColor;

	cv::cvtColor(img, imgColor, cv::COLOR_GRAY2BGR);
	imgColor.convertTo(imgColor, CV_8UC3, 255.0);
	detector.showResult("cor", QRs, imgColor);
	return;
}


int main()
{
	testCornerDetect();

	return 0;
}