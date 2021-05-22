#include "Detector.h"
using namespace cv;
using namespace std;

CDetector* g_pDetector = NULL;
bool g_bFirstFrame = true;

int g_nMinWidthContour = 100;
int g_nMinHegihtContour = 100;

CDetector::CDetector(Mat& gray)
{
	fg = gray.clone();
	bs = new BackgroundSubtract();
	bs->init(gray);
	//vector<Rect> rects;
	//vector<Point2d> centers;
}

CDetector::~CDetector(void)
{
	delete bs;
	bs = NULL;
}

//----------------------------------------------------------------------
// Detector
//----------------------------------------------------------------------
void CDetector::DetectContour(Mat& img, vector<Rect>& Rects, vector<Point2d>& centers)
{
	double area = 0;

	Rects.clear();
	centers.clear();

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	Mat edges = img.clone();

	Canny(img, edges, 50, 190, 3);
	findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());

	if(contours.size() >= 1)
	{
		for(unsigned int i = 0; i < contours.size(); i++ )
		{
			Rect r = cv::boundingRect(contours[i]);
			if (r.width > g_nMinWidthContour && r.height > g_nMinHegihtContour)
			{
				Rects.push_back(r);
				centers.push_back((r.br()+r.tl())*0.5);
			}
		}
	}
}

vector<Point2d> CDetector::Detect(Mat& gray)
{
	if( bs != NULL )
	{
		//imshow("Video", fg);
		bs->subtract(gray, fg);
			// rects - bounding rectangles
			// centers - centers of bounding rectangles
			/*
			Mat fg2;
			fg.convertTo(fg2,CV_32FC1);
			cv::GaussianBlur(fg2,fg2,Size(5,5),1.0);
			cv::Laplacian(fg2,fg2,CV_32FC1);

			normalize(fg2,fg2,0,255,cv::NORM_MINMAX);
			fg2.convertTo(fg2,CV_8UC1);
			cv::applyColorMap(fg2,fg2,COLORMAP_JET);*/
			//imshow("Foreground", fg);
			
			DetectContour(fg, m_arrRects, m_arrCenters);
	}

	return m_arrCenters;
}
