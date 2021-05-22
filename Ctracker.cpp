#include "Ctracker.h"
//#include "AssignmentOptimal.h"

using namespace cv;
using namespace std;

CTracker *g_pTracker = NULL;
size_t CTracker::m_nTrackCount = 0;
size_t g_nNextTrackID = 0;

// ---------------------------------------------------------------------------
// Track constructor.
// The track begins from initial point (pt)
// ---------------------------------------------------------------------------
CTrack::CTrack(Rect rtTrack, Point2f pt, float dt, float Accel_noise_mag, int NextTrackID)
{
	track_id = NextTrackID;
	// Every track have its own Kalman filter,
	// it user for next point position prediction.
	KF = new TKalmanFilter(pt, dt, Accel_noise_mag);
	// Here stored points coordinates, used for next position prediction.
	prediction = pt;
	ptStart = pt;
	skipped_frames = 0;

	m_rtTrack = rtTrack;
	m_bCounted = false;
	m_nDirection = 0;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTrack::~CTrack()
{
	// Free resources.
	delete KF;
	KF = NULL;
}

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
CTracker::CTracker(float _dt, float _Accel_noise_mag, double _dist_thres, unsigned int _maximum_allowed_skipped_frames, unsigned int _max_trace_length)
{
	dt = _dt;
	Accel_noise_mag = _Accel_noise_mag;
	dist_thres = _dist_thres;
	max_trace_length = _max_trace_length;
	maximum_allowed_skipped_frames = _maximum_allowed_skipped_frames;

	m_nTrackCount = 0;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::Update(vector<Rect>& arrRects, vector<Point2d>& ptCenters, Mat matImg)
{
	unsigned int i, j;
	// -----------------------------------
	// If there is no tracks yet, then every point begins its own track.
	// -----------------------------------
	if(tracks.size() == 0)
	{
		// If no tracks yet
		for(i = 0; i < ptCenters.size(); i++)
		{
			CTrack* tr = new CTrack(arrRects[i], ptCenters[i], dt, Accel_noise_mag, g_nNextTrackID++);
			tracks.push_back(tr);
		}	
	}

	// -----------------------------------
	// 
	// -----------------------------------
	int N = tracks.size();		// 
	int M = ptCenters.size();	// 
	// N - , M-
	vector< vector<double> > Cost(N, vector<double>(M));
	vector<int> assignment; //

	// -----------------------------------
	// 
	// -----------------------------------
	double dist;
	for(i = 0; i < tracks.size();i++)
	{	
		// Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for(j = 0; j < ptCenters.size();j++)
		{
			Point2d diff = (tracks[i]->prediction - ptCenters[j]);
			dist = (double)sqrtf((float)(diff.x*diff.x + diff.y*diff.y));
			Cost[i][j] = dist;
		}
	}
	// -----------------------------------
	// Solving assignment problem (tracks and predictions of Kalman filter)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost, assignment, AssignmentProblemSolver::optimal);
	//AssignmentOptimal APS;
	//APS.Solve(Cost, assignment);

	// -----------------------------------
	// clean assignment from pairs with large distance
	// -----------------------------------
	// Not assigned tracks
	vector<int> not_assigned_tracks;
	for(i = 0; i < assignment.size(); i++)
	{
		if(assignment[i] != -1)
		{
			if(Cost[i][assignment[i]] > dist_thres)
			{
				assignment[i] = -1;
				// Mark unassigned tracks, and increment skipped frames counter,
				// when skipped frames counter will be larger than threshold, track will be deleted.
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{			
			// If track have no assigned detect, then increment skipped frames counter.
			tracks[i]->skipped_frames++;
		}

	}

	// -----------------------------------
	// If track didn't get detects long time, remove it.
	// -----------------------------------
	for(i = 0; i < tracks.size(); i++)
	{
		if(tracks[i]->skipped_frames > maximum_allowed_skipped_frames)
		{
			delete tracks[i];
			tracks.erase(tracks.begin()+i);
			assignment.erase(assignment.begin()+i);
			i--;
		}
	}
	// -----------------------------------
	// Search for unassigned detects
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for(i = 0; i < ptCenters.size(); i++)
	{
		it = find(assignment.begin(), assignment.end(), i);
		if(it == assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	// -----------------------------------
	// and start new tracks for them.
	// -----------------------------------
	if(not_assigned_detections.size() != 0)
	{
		for(i = 0; i < not_assigned_detections.size(); i++)
		{
			CTrack* tr = new CTrack(arrRects[not_assigned_detections[i]], ptCenters[not_assigned_detections[i]], dt, Accel_noise_mag, g_nNextTrackID++);
			tracks.push_back(tr);
		}	
	}

	// Update Kalman Filters state
	for(i = 0; i < assignment.size();i++)
	{
		// If track updated less than one time, than filter state is not correct.
		tracks[i]->KF->GetPrediction();

		if(assignment[i] !=-1 )
		{// If we have assigned detect, then update using its coordinates,
			tracks[i]->skipped_frames = 0;
			tracks[i]->prediction = tracks[i]->KF->Update(ptCenters[assignment[i]], 1);
		}
		else
		{// if not continue using predictions
			tracks[i]->prediction = tracks[i]->KF->Update(Point2f(0,0), 0);	
		}
		
		if(tracks[i]->trace.size() > max_trace_length)
		{
			tracks[i]->trace.erase(tracks[i]->trace.begin(), tracks[i]->trace.end() - max_trace_length);
		}

		tracks[i]->trace.push_back(tracks[i]->prediction);
		tracks[i]->KF->LastResult = tracks[i]->prediction;
	}

	for(i = 0; i < assignment.size(); i++)
	{
		if(assignment[i] != -1)
		{
			//Point2d pt1 = new Point(rects
			CTrack *pTrack = tracks[i];
			//if( pTrack->m_rtTrack.width != 0 && pTrack->m_rtTrack.height != 0  && pTrack->m_bCounted == false )
			//{
			//	pTrack->m_bCounted = true;
			//	m_nTrackCount++;
			//}
			pTrack->m_nDirection = 0;
			for(j = 0; j < pTrack->trace.size()-1; j++)
			{
				pTrack->m_nDirection += (int)( pTrack->trace[j+1].x - pTrack->trace[j].x );
			}
		}
	}
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTracker::~CTracker(void)
{
	unsigned int i;
	for(i = 0; i < tracks.size();i++)
	{
		delete tracks[i];
	}
	tracks.clear();
}
