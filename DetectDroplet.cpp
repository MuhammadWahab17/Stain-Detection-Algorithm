#include "opencv2/opencv.hpp"

#include "Detector.h"
#include "Ctracker.h"

#ifdef _DEBUG
#define LOG_VIEW

#pragma comment (lib, "opencv_world420d.lib")

#else

#pragma comment (lib, "opencv_world420.lib")

#endif

using namespace cv;
using namespace std;

#define COLOR_OUTER_WRAP	Scalar(255, 0, 0)
#define COLOR_INNER_DROP	Scalar(0, 0, 255)
#define COLOR_OUTER_CEN		Scalar(255, 0, 255)
#define COLOR_INNER_CEN		Scalar(0, 255, 0)

int  g_nOffCross;
int  g_nOffHightPassed;
int			m_nCountingEnter;
int			m_nCountingOut;
int			m_nCountingEnterPre;
int			m_nCountingOutPre;

void findContours(Mat matBin)
{
	// contours vector  
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// find contours for the thresholded image
	findContours(matBin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));



}
Rect getRectFromCircle(Vec3i circle)
{
	Point center = Point(circle[0], circle[1]);
	int radius = circle[2];
	Rect rt;
	rt.x = center.x - radius;
	rt.width = 2 * radius;
	rt.y = center.y - radius;
	rt.height = 2 * radius;
	return rt;
}
vector<Rect> getOverlapCircles(vector<Vec3f> &circles)
{
	vector<Rect> vtRect;
	int num_circles = (int)circles.size();
	bool* pOver = new bool[num_circles];
	memset(pOver, 0, num_circles * sizeof(bool));
	for (int i = 0; i < num_circles; i++)
	{
		if ( pOver[i] ) continue;
		Rect rtI = getRectFromCircle(circles[i]);
		for (int j = 0; j < num_circles; j++)
		{
			if (i == j) continue;
			if (pOver[j]) continue;
			Rect rtJ = getRectFromCircle(circles[j]);
			int max_l = rtI.x;
			int max_t = rtI.y;
			int min_r = rtI.br().x;
			int min_b = rtI.br().y;
			if (rtJ.x > max_l) max_l = rtJ.x;
			if (rtJ.y > max_t) max_t = rtJ.y;
			if (rtJ.br().x < min_r) min_r = rtJ.br().x;
			if (rtJ.br().y < min_b) min_b = rtJ.br().y;

			if (2 * (min_r - max_l) > rtI.width || 2 * (min_r - max_l) > rtJ.width)
			{
				if (2 * (min_b - max_t) > rtI.height || 2 * (min_b - max_t) > rtJ.height)
				{
					int right = rtI.x + rtI.width;
					int bottom = rtI.y + rtI.height;
					if (rtJ.x < rtI.x) rtI.x = rtJ.x;
					if (rtJ.y < rtI.y) rtI.y = rtJ.y;
					if (rtJ.x + rtJ.width > right)
						rtI.width = rtJ.x + rtJ.width - rtI.x;
					if (rtJ.y + rtJ.height > bottom)
						rtI.height = rtJ.y + rtJ.height - rtI.y;
					pOver[j] = true;
				}
			}

		}

		vtRect.push_back(rtI);
	}

	delete[] pOver; pOver = NULL;

	return vtRect;
}

bool cmpCircleArea(Rect rt1, Rect rt2)
{
	bool ret = false;
	int area1 = rt1.width * rt1.height;
	int area2 = rt2.width * rt2.height;
	if (area1 < area2)
		ret = true;

	return ret;
}
bool cmpRectX(Rect rt1, Rect rt2)
{
	bool ret = false;
	if (rt1.x < rt2.x)
		ret = true;

	return ret;
}
bool IsCircle(Mat gray_org, Rect rt, Rect& rtC)
{
	bool ret = false;
	Mat gray = gray_org(rt);

	Mat blue;
	medianBlur(gray, blue, 5);
	vector<Vec3f> circles;
	HoughCircles(blue, circles, HOUGH_GRADIENT, 1,
		gray.rows / 16,  // change this value to detect circles with different distances to each other
		100, 30, 1, 30 // change the last two parameters
		// (min_radius & max_radius) to detect larger circles
	);

	if (circles.size() == 0)
		return ret;

	ret = true;
	vector<Rect> vtRtCircles = getOverlapCircles(circles);
	sort(vtRtCircles.begin(), vtRtCircles.end(), cmpCircleArea);
	rtC.x = rt.x + vtRtCircles.at(0).x;
	rtC.y = rt.y + vtRtCircles.at(0).y;
	rtC.width = vtRtCircles.at(0).width;
	rtC.height = vtRtCircles.at(0).height;
	return ret;
}
void MergeRects(vector<Rect>& vtRtIn)
{
	vector<Rect> vtRtOut;
	int num_rt = (int)vtRtIn.size();
	bool* over = new bool[num_rt];
	memset(over, 0, num_rt * sizeof(bool));

	sort(vtRtIn.begin(), vtRtIn.end(), cmpRectX);
	for (int i = 0; i < num_rt; i++)
	{
		if (over[i]) continue;
		Rect rtI = vtRtIn.at(i);

		for (int j = i + 1; j < num_rt; j++)
		{
			if (over[j]) continue;
			Rect rtJ = vtRtIn.at(j);

			int max_l = rtI.x;
			int max_t = rtI.y;
			int min_r = rtI.br().x;
			int min_b = rtI.br().y;
			if (rtJ.x > max_l) max_l = rtJ.x;
			if (rtJ.y > max_t) max_t = rtJ.y;
			if (rtJ.br().x < min_r) min_r = rtJ.br().x;
			if (rtJ.br().y < min_b) min_b = rtJ.br().y;

			bool merge = false;
			if (abs(rtI.x + rtI.width / 2 - rtJ.x - rtJ.width / 2) < min(rtI.height, rtJ.height))
				merge = true;

			if (2 * (min_r - max_l) > rtI.width || 2 * (min_r - max_l) > rtJ.width)
			{
				if (2 * (min_b - max_t) > rtI.height || 2 * (min_b - max_t) > rtJ.height)
					merge = true;
			}
			if( merge)
			{
				int right = rtI.x + rtI.width;
				int bottom = rtI.y + rtI.height;
				if (rtJ.x < rtI.x) rtI.x = rtJ.x;
				if (rtJ.y < rtI.y) rtI.y = rtJ.y;
				if (rtJ.x + rtJ.width > right)
					rtI.width = rtJ.x + rtJ.width - rtI.x;
				if (rtJ.y + rtJ.height > bottom)
					rtI.height = rtJ.y + rtJ.height - rtI.y;
				over[j] = true;
			}
		}

		vtRtOut.push_back(rtI);
	}

	vtRtIn.clear();
	vtRtIn = vtRtOut;
}
int main_temp( int argc, char** argv )
{
	// Create a VideoCapture object and open the input file
	// If the input is the web camera, pass 0 instead of the video file name
	//VideoCapture cap(argv[1]);
	VideoCapture cap("E:\\_work\\fl\\DetectDrop\\DetectDroplet\\x64\\Debug\\video_2.avi");
	VideoWriter videoW;

	int count_outer = 0;
	int count_inner = 0;
	int count_frame = 0;

	int first_inner_pos = 0;

	// Check if camera opened successfully
	if (!cap.isOpened()) {
		cout << "Error opening video stream or file" << endl;
		return -1;

	}

	double fps = 20.0;
	Size sizeFrame = Size((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));
	//if(argv[2] != NULL)
	videoW.open(string("output_result.avi"), VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, sizeFrame);
	
	while (1) {

		Mat img;
		// Capture frame-by-frame
		cap >> img;

		// If the frame is empty, break immediately
		if (img.empty())
			break;


		Mat imgGray, imgBlur, imgCanny, imgDil, imgErode;// , imgHSV;

		// Preprocessing
		cvtColor(img, imgGray, COLOR_BGR2GRAY);


		Mat gray;
		medianBlur(imgGray, gray, 5);
		vector<Vec3f> circles;
		HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
			gray.rows / 16,  // change this value to detect circles with different distances to each other
			100, 30, 1, 30 // change the last two parameters
			// (min_radius & max_radius) to detect larger circles
		);
		//for (size_t i = 0; i < circles.size(); i++)
		//{
		//	Vec3i c = circles[i];
		//	Point center = Point(c[0], c[1]);
		//	// circle center
		//	circle(img, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
		//	// circle outline
		//	int radius = c[2];
		//	circle(img, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
		//}

		//============================================//
		//cvtColor(img, imgHSV, COLOR_BGR2HSV);

		GaussianBlur(imgGray, imgBlur, Size(3, 3), 3, 0);
		Canny(imgBlur, imgCanny, 25, 75);

		Mat bw;
		adaptiveThreshold(~gray, bw, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 31, -2);
		// Show binary image
		//cv::imshow("BW", bw);

		// Create the images that will use to extract the horizontal lines
		Mat horizontal = bw.clone();
		// Specify size on horizontal axis
		int horizontal_size = horizontal.cols / 30;
		// Create structure element for extracting horizontal lines through morphology operations
		Mat horizontalStructure = getStructuringElement(MORPH_RECT, Size(horizontal_size, 1));
		// Apply morphology operations
		erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
		dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));
		// Show extracted horizontal lines

		// Create the images that will use to extract the vertical lines
		Mat vertical = bw.clone();
		// Specify size on vertical axis
		int vertical_size = vertical.rows / 30;
		// Create structure element for extracting vertical lines through morphology operations
		Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, vertical_size));
		// Apply morphology operations
		erode(vertical, vertical, verticalStructure, Point(-1, -1));
		dilate(vertical, vertical, verticalStructure, Point(-1, -1));
		// Show extracted vertical lines
		//imshow("vertical", vertical);

		//======================================================//
		// Inverse vertical image
		bitwise_not(vertical, vertical);

		// Extract edges and smooth image according to the logic
		// 1. extract edges
		// 2. dilate(edges)
		// 3. src.copyTo(smooth)
		// 4. blur smooth img
		// 5. smooth.copyTo(src, edges)
		// Step 1
		Mat edges;
		adaptiveThreshold(vertical, edges, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 31, -2);

		// Step 2
		Mat kernel = Mat::ones(2, 2, CV_8UC1);
		dilate(edges, edges, kernel);

		// Step 3
		Mat smooth;
		vertical.copyTo(smooth);
		// Step 4
		blur(smooth, smooth, Size(2, 2));
		// Step 5
		smooth.copyTo(vertical, edges);

		Mat matDraw = img.clone();
		putText(matDraw, "Blue : Outer wrap", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.5, COLOR_OUTER_WRAP, 2);
		putText(matDraw, "Red : Inner Drop", Point(400, 20), FONT_HERSHEY_SIMPLEX, 0.5, COLOR_INNER_DROP, 2);
		putText(matDraw, "Yello : Outer centre", Point(100, matDraw.size().height - 30), FONT_HERSHEY_SIMPLEX, 0.5, COLOR_OUTER_CEN, 2);
		putText(matDraw, "Green : Inner centre", Point(400, matDraw.size().height - 30), FONT_HERSHEY_SIMPLEX, 0.5, COLOR_INNER_CEN, 2);
		//cvtColor(vertical, matDraw, COLOR_GRAY2BGR);

		char msg_count[256];
		memset(msg_count, 0, 256);
		sprintf_s(msg_count, "Inner Drop Counting : %d", count_inner);
		putText(matDraw, msg_count, Point(200, 50), FONT_HERSHEY_SIMPLEX, 1.0, COLOR_INNER_CEN, 2);

		//Display the cirlcle
		if (circles.size() > 0) {
			// contours vector  
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			//Mat matROI = vertical(rtROI);
			// find contours for the thresholded image
			findContours(vertical, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

			// circle contours
			//vector< vector<Point> > ctrCircles;

			vector<Rect> vtRtDrop = getOverlapCircles(circles);
			Rect rtROI = vtRtDrop[0];
			for (size_t i = 0; i < vtRtDrop.size(); i++)
			{
				Rect rtCir = vtRtDrop[i];
				bool bOuter = false;
				bool bInner = false;
				int idx_outer = -1;
				int idx_inner = -1;

				Point ptOuter, ptInner;
				Rect rtOuter, rtInner;
				rtOuter = boundingRect(contours[0]);
				for (int j = 0; j < contours.size(); j++)
				{
					Rect rtC = boundingRect(contours[j]);
					if (rtC.x < rtCir.x - 10 || rtC.br().x > rtCir.br().x + 10 )
						continue;
					if (rtC.y < rtCir.y - 10 || rtC.br().y > rtCir.br().y + 10)
						continue;

					if (4 * rtC.width < rtCir.width || 4 * rtC.height < rtCir.height)
						continue;

					Moments mu;
					if (2 * abs(rtC.width - rtCir.width) > 1 * rtCir.width)
					{
						bInner = true;
						rtInner = rtC;
						idx_inner = j;
						mu = moments(contours[j], false);
						int cx = (int)(mu.m10 / mu.m00);
						int cy = (int)(mu.m01 / mu.m00);
						ptInner = Point(cx, cy);
					}
					else
					{
						bOuter = true;
						if (rtOuter.width < rtC.width)
						{
							rtOuter = rtC;
							idx_outer = j;
							mu = moments(contours[j], false);
							int cx = (int)(mu.m10 / mu.m00);
							int cy = (int)(mu.m01 / mu.m00);

							ptOuter = Point(cx, cy);
						}
					}

					//ctrCircles.push_back(contours[j]);
				}

				if (bOuter && idx_outer > 0) {
					drawContours(matDraw, contours, idx_outer, COLOR_OUTER_WRAP, 2, 8, vector<Vec4i>(), 0, Point());
					circle(matDraw, ptOuter, 2, COLOR_OUTER_CEN, 3, LINE_AA);
				}
				else
				{
					Point center = Point(rtCir.x + rtCir.width / 2, rtCir.y + rtCir.height / 2);
					// circle outline
					int radius = rtCir.width / 2;
					circle(matDraw, center, radius, COLOR_OUTER_WRAP, 3, LINE_AA);
					// circle center
					circle(matDraw, center, 2, COLOR_OUTER_CEN, 3, LINE_AA);
				}

				if (bInner && idx_inner > 0) {
					int w2 = img.size().width / 4;
					int radius = rtInner.width / 2;
					Point center = Point(rtInner.x + radius, rtInner.y + radius);
					circle(matDraw, center, radius, COLOR_INNER_DROP, 3, LINE_AA);
					circle(matDraw, ptInner, 2, COLOR_INNER_CEN, 3, LINE_AA);
					//drawContours(matDraw, contours, idx_inner, Scalar(255, 0, 0), 2, 8, vector<Vec4i>(), 0, Point());
					if (rtInner.x > w2 - 5 && rtInner.x < w2 + 5)
					{
						if (count_frame == 0) {
							count_inner++;
							count_frame++;
						}
						else if(count_frame > 0)
							count_frame++;

						if (count_frame > 10)
							count_frame = 0;
					}
				}

				//int right = rtROI.x + rtROI.width;
				//int bottom = rtROI.y + rtROI.height;
				//if (rt.x < rtROI.x) rtROI.x = rt.x;
				//if (rt.y < rtROI.y) rtROI.y = rt.y;
				//if (rt.x + rt.width > right) rtROI.width = rt.x + rt.width;
				//if (rt.y + rt.height > bottom) rtROI.height = rt.y + rt.height; 
			}



			// draw contours and convex hull on the empty black image 
			//for (int i = 0; i < ctrCircles.size(); i++) {
			//	//Scalar color_contours = Scalar(0, 255, 255); // color for contours : blue
			//	//Scalar color = Scalar(255, 255, 255); // color for convex hull : white
			//	// draw contours
			//	drawContours(matDraw, ctrCircles, i, Scalar(0, 0, 255), 2, 8, vector<Vec4i>(), 0, Point());
			//	// draw convex hull
			//	//drawContours(drawing, hull, i, color, 2, 8, vector<Vec4i>(), 0, Point());
			//}

			//rectangle(matDraw, rtROI, Scalar(0, 0, 255), 3);
		}
		// Show final result
		///cv::imshow("smooth - final", matDraw);
		cv::imshow("detected circles", matDraw);

		// Display the resulting frame
		//imshow("GrayScale Video", imgGray);

		//imshow("Image Canny", imgCanny);
		//=======================================================//

		videoW.write(matDraw);

		// Press  ESC on keyboard to exit
		char c = (char)waitKey(25);
		if (c == 27)
			break;

	}
	// When everything done, release the video capture object
	cap.release();
	videoW.release();
	// Closes all the frames
	cv::destroyAllWindows();

	return 0;
}

bool Engine_Init(int nMaxWidthContour, int nMaxHeightContour)
{
	//g_pTracker = new CTracker(0.2, 0.5, 60.0, 10, 10);
	g_pTracker = new CTracker(0.2f, 0.5f, 600.0, 10, 10);

	g_bFirstFrame = true;
	g_nMinWidthContour = nMaxWidthContour;
	g_nMinHegihtContour = nMaxHeightContour;

	if (g_pTracker == NULL)
		return false;

	g_nNextTrackID = 0;
	g_nOffCross = 20;
	g_nOffHightPassed = 150;

	return true;
}

void Engine_Release()
{
	if (g_pTracker != NULL)
	{
		delete g_pTracker;
		g_pTracker = NULL;
	}
	if (g_pDetector != NULL)
	{
		delete g_pDetector;
		g_pDetector = NULL;
	}
}

bool Engine_ProcPeopleCounting(Mat& matImg, double dRateCross, int& nNumEnter, int& nNumOut)
{
	//bool bRet = ProcOCR(pbyImgRGB, nImgW, nImgH, (OCR_INFO *)pInfoOCR);
	bool bRet = false;//ProcOCR(pbyImgRGB, nImgW, nImgH, pszLetters, nNumLetters);

	if (g_pTracker == NULL)
	{
		printf("g_pTracker == NULL");
		return bRet;
	}
	if (g_pDetector == NULL)
	{
		printf("g_pDetector == NULL");
		return bRet;
	}

	Mat matGray, gray;
	cvtColor(matImg, matGray, cv::COLOR_BGR2GRAY);
	gray = matGray.clone();
	vector<Point2d> centers = g_pDetector->Detect(gray);
	MergeRects(g_pDetector->m_arrRects);

	int left, top, w, h;
	vector<Rect> vtRtCircles;
	vector<Point2d> vtPtCenters;
	for (int i = 0; i < (int)g_pDetector->m_arrRects.size(); i++)
	{
		//rectangle(matImg, g_pDetector->rects[i], Scalar(0, 255, 0), 2);
		//left = (int)centers[i].x - 50; if (left < 0) left = 0;
		//top = (int)centers[i].y - 50; if (top < 0) top = 0;
		//w = 100; if (left + w >= matImg.size().width) w = matImg.size().width - left - 1;
		//h = 100; if (top + h >= matImg.size().height) h = matImg.size().height - top - 1;
		//rectangle(matImg, Rect(left, top, w, h), Scalar(0, 255, 0), 2);
		
		Rect rtObj = g_pDetector->m_arrRects.at(i);
		rectangle(matImg, rtObj, Scalar(0, 255, 0), 2);
		Rect rtC;
		if (IsCircle(gray, rtObj, rtC))
		{
			vtRtCircles.push_back(rtC);
			vtPtCenters.push_back(Point(rtC.x + rtC.width / 2, rtC.y + rtC.height / 2));
		}
	}
	//imshow("Sub", fg);
	//for(int i = 0; i < centers.size(); i++)
	//{
	//	circle(frame, centers[i], 3, Scalar(0,255,0), 1, CV_AA);
	//}

	size_t nNumTrack = 0;
	int nCrossHeight = (int)(dRateCross * matImg.size().width);
	if (vtPtCenters.size() > 0)
	{
		//g_pTracker->Update(g_pDetector->m_arrRects, g_pDetector->m_arrCenters, matImg);
		g_pTracker->Update(vtRtCircles, vtPtCenters, matImg);

		//cout << tracker.tracks.size()  << endl;
		//cout << tracker.tracks.NextTrackID << endl;
		//printf("tracks size == %d \n", g_pTracker->tracks.size());

		char szTrackNum[20];
		for (unsigned int i = 0; i < g_pTracker->tracks.size(); i++)
		{
			//putText(matImg, "text", g_pDetector->m_arrCenters[i], FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0, Scalar::all(255), 2);
			nNumTrack = g_pTracker->tracks[i]->trace.size();
			if (nNumTrack > 0)
			{
				for (unsigned int j = 0; j < nNumTrack - 1; j++)
				{
					Point pt0 = g_pTracker->tracks[i]->trace[j];
					Point pt1 = g_pTracker->tracks[i]->trace[j + 1];
					int clr = (g_pTracker->tracks[i]->track_id % 5) * 50;
					line(matImg, pt0, pt1, Scalar(clr, 0, 0), 2);
				}
				Point center = g_pTracker->tracks[i]->prediction;
				int radius = g_pTracker->tracks[i]->m_rtTrack.width / 2;
				circle(matImg, center, 2, COLOR_OUTER_CEN, 3, LINE_AA);
				circle(matImg, center, radius, COLOR_OUTER_WRAP, 3, LINE_AA);

				memset(szTrackNum, 0, 20 * sizeof(char));
				int idx_track = (int)g_pTracker->tracks[i]->track_id;
				sprintf_s(szTrackNum, 20, "%d", idx_track);
				putText(matImg, szTrackNum, center, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);

				if (nNumEnter < idx_track)
					nNumEnter = idx_track;

				//if (g_pTracker->tracks[i]->m_bCounted == false)
				//{
				//	if (g_pTracker->tracks[i]->m_nDirection - 10 > 0 &&
				//		//g_pTracker->tracks[i]->trace[nNumTrack-1].y > nCrossHeight && g_pTracker->tracks[i]->trace[nNumTrack-1].y < nCrossHeight + g_nOffHightPassed - g_nOffCross )
				//		g_pTracker->tracks[i]->prediction.x > nCrossHeight && g_pTracker->tracks[i]->prediction.x < nCrossHeight + g_nOffHightPassed - g_nOffCross)
				//	{
				//		nNumEnter++;
				//		g_pTracker->tracks[i]->m_bCounted = true;
				//	}
				//}

				//if (g_pTracker->tracks[i]->m_bCounted == true)
				//{
				//	//if( g_pTracker->tracks[i]->m_nDirection - 10 > 0 && g_pTracker->tracks[i]->trace[nNumTrack-1].y > nCrossHeight + g_nOffHightPassed)
				//	if (g_pTracker->tracks[i]->m_nDirection - 10 > 0 && g_pTracker->tracks[i]->prediction.x > nCrossHeight + g_nOffHightPassed)
				//	{
				//		g_pTracker->tracks[i]->m_bCounted = false;
				//	}
				//	//else if(g_pTracker->tracks[i]->m_nDirection + 10 < 0 && g_pTracker->tracks[i]->trace[nNumTrack-1].y < nCrossHeight - g_nOffHightPassed )
				//	else if (g_pTracker->tracks[i]->m_nDirection + 10 < 0 && g_pTracker->tracks[i]->prediction.x < nCrossHeight - g_nOffHightPassed)
				//	{
				//		g_pTracker->tracks[i]->m_bCounted = false;
				//	}
				//}
			}
		}
	}

	return bRet;
}

int main(int argc, char** argv)
{
	// Create a VideoCapture object and open the input file
	// If the input is the web camera, pass 0 instead of the video file name
	//VideoCapture cap(argv[1]);
	VideoCapture cap("E:\\_work\\fl\\DetectDrop\\DetectDroplet\\x64\\Debug\\video_2.avi");
	VideoWriter videoW;

	int count_outer = 0;
	int count_inner = 0;
	int count_frame = 0;

	int first_inner_pos = 0;

	// Check if camera opened successfully
	if (!cap.isOpened()) {
		cout << "Error opening video stream or file" << endl;
		return -1;

	}

	double fps = 20.0;
	Size sizeFrame = Size((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));
	//if(argv[2] != NULL)
	videoW.open(string("output_result.avi"), VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, sizeFrame);


	if (Engine_Init(50, 50) == false)
	{
		printf("can not create the CTracker clasee. \n");
	}



	while (1) {

		Mat img;
		// Capture frame-by-frame
		cap >> img;

		// If the frame is empty, break immediately
		if (img.empty())
			break;

		Mat matDraw = img.clone();
		Mat imgGray, imgBlur, imgCanny, imgDil, imgErode;// , imgHSV;


		if (g_pDetector == NULL)
		{
			Mat imgFrameGray;
			cvtColor(img, imgFrameGray, cv::COLOR_BGR2GRAY);
			g_pDetector = new CDetector(imgFrameGray);
		}

		Engine_ProcPeopleCounting(matDraw, 0.5, m_nCountingEnter, m_nCountingOut);


		char msg_count[256];
		memset(msg_count, 0, 256);
		sprintf_s(msg_count, "Inner Drop Counting : %d", m_nCountingEnter);
		putText(matDraw, msg_count, Point(200, 20), FONT_HERSHEY_SIMPLEX, 1.0, COLOR_OUTER_WRAP, 2);

		// Show final result
		///cv::imshow("smooth - final", matDraw);
		cv::imshow("detected circles", matDraw);

		// Display the resulting frame
		//imshow("GrayScale Video", imgGray);

		//imshow("Image Canny", imgCanny);
		//=======================================================//

		videoW.write(matDraw);

		// Press  ESC on keyboard to exit
		char c = (char)waitKey(25);
		if (c == 27)
			break;

	}
	// When everything done, release the video capture object
	cap.release();
	videoW.release();
	// Closes all the frames
	cv::destroyAllWindows();

	Engine_Release();

	return 0;
}






































+923180629837
whatsapp










