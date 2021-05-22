//#pragma once

#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include "opencv2/opencv.hpp"
#include "BackgroundSubtract.h"
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

class CDetector
{
private:
	Mat fg;
	BackgroundSubtract* bs;

	void DetectContour(Mat& img, vector<Rect>& Rects,vector<Point2d>& centers);

public:
	vector<Rect>	m_arrRects;
	vector<Point2d> m_arrCenters;

	CDetector(Mat& gray);
	vector<Point2d> Detect(Mat& gray);
	~CDetector(void);
};

extern CDetector*	g_pDetector;
extern bool			g_bFirstFrame;

extern int			g_nMinWidthContour;
extern int			g_nMinHegihtContour;

#endif //_DETECTOR_H_

