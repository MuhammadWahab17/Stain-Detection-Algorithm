
//#pragma once

#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "Kalman.h"
#include "HungarianAlg.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

class CTrack
{
public:
	vector<Point2d>		trace;
	size_t				track_id;
	size_t				skipped_frames; 
	Point2d				prediction;
	Point2d				ptStart;
	TKalmanFilter		*KF;

	bool				m_bCounted;
	Rect				m_rtTrack;
	int					m_nDirection;

	CTrack(Rect rtTrack, Point2f p, float dt, float Accel_noise_mag, int NextTrackID);
	~CTrack();
};


class CTracker
{
public:
	
	float dt; 
	float Accel_noise_mag;
	double dist_thres;
	unsigned int maximum_allowed_skipped_frames, max_trace_length;
	
	vector<CTrack*> tracks;
	static size_t	m_nTrackCount;
	static size_t	NextTrackID;

	void Update(vector<Rect>& arrRects, vector<Point2d>& ptCenters, Mat matImg);

	CTracker(float _dt, float _Accel_noise_mag, double _dist_thres=60, unsigned int _maximum_allowed_skipped_frames = 5, unsigned int _max_trace_length = 10);
	~CTracker(void);
};

extern CTracker *g_pTracker;
extern size_t g_nNextTrackID;

#endif //_TRACKER_H_
