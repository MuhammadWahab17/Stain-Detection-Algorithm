#include "BackgroundSubtract.h"

BackgroundSubtract::BackgroundSubtract()
{
	model = libvibeModelNew();
}

BackgroundSubtract::~BackgroundSubtract()
{
	libvibeModelFree(model);
	delete model;
	model = NULL;
}

void BackgroundSubtract::init(cv::Mat &image)
{
	int32_t width = image.size().width;
	int32_t height = image.size().height;
	int32_t stride = image.channels()*image.size().width;
	uint8_t *image_data = (uint8_t*)image.data;

	libvibeModelInit(model, image_data, width, height, stride);
}

void BackgroundSubtract::subtract(const cv::Mat &image, cv::Mat &foreground)
{
	if( model == NULL)
		return;

	uint8_t *image_data = (uint8_t*)image.data;
	uint8_t *segmentation_map = (uint8_t*)foreground.data;

	cv::Mat eElement = cv::getStructuringElement( 0, cv::Size( 1, 3 ), cv::Point( -1, -1 ) );
	cv::Mat dElement = cv::getStructuringElement(0, cv::Size(25, 35), cv::Point(-1, -1));
	cv::Mat erodeElement1 = cv::getStructuringElement( 0, cv::Size( 5, 5 ), cv::Point( -1, -1 ) );
	cv::Mat dilateElement1 = cv::getStructuringElement(0, cv::Size(10, 10), cv::Point(-1, -1));
	cv::Mat dilateElementY = cv::getStructuringElement(0, cv::Size(45, 40), cv::Point(-1, -1));

	libvibeModelUpdate(model, image_data, segmentation_map);

	cv::erode(foreground, foreground, eElement, cv::Point(-1, -1), 1);
	cv::dilate(foreground, foreground, dElement, cv::Point(-1, -1), 1);
	//cv::dilate(foreground, foreground, dilateElementY, cv::Point(-1, -1), 1);

	//cv::erode(foreground, foreground, dilateElement, cv::Point(-1, -1), 1);

	//cv::dilate(foreground, foreground, dilateElementY, cv::Point(-1, -1), 1);

	////cv::dilate(foreground, foreground, dilateElement, cv::Point(-1,-1), 1);
	//cv::erode(foreground, foreground, erodeElement, cv::Point(-1, -1), 1);
	//cv::dilate(foreground, foreground, dilateElement, cv::Point(-1,-1), 2);

	//cv::erode(foreground, foreground, dilateElement, cv::Point(-1, -1), 1);
	//cv::dilate(foreground, foreground, dilateElement, cv::Point(-1, -1), 1);
}