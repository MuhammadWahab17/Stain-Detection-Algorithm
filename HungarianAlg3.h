
#ifndef _HUNGARIAN_ALG3_H_
#define _HUNGARIAN_ALG3_H_

#include <vector>
#include <iostream>
#include <limits>
#include <time.h>
// http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm
using namespace std;


class HungarianAlg3
{
	//private final double[][] costMatrix;
	//private final int rows, cols, dim;
	//private final double[] labelByWorker, labelByJob;
	//private final int[] minSlackWorkerByJob;
	//private final double[] minSlackValueByJob;
	//private final int[] matchJobByWorker, matchWorkerByJob;
	//private final int[] parentWorkerByCommittedJob;
	//private final boolean[] committedWorkers;
private:
	double *costMatrix;
	int rows, cols, dim;
	double *labelByWorker, *labelByJob;
	int *minSlackWorkerByJob;
	double *minSlackValueByJob;
	int *matchJobByWorker, *matchWorkerByJob;
	int *parentWorkerByCommittedJob;
	bool *committedWorkers;

	/**
	 * Construct an instance of the algorithm.
	 * 
	 * @param costMatrix
	 *            the cost matrix, where matrix[i][j] holds the cost of
	 *            assigning worker i to job j, for all i, j. The cost matrix
	 *            must not be irregular in the sense that all rows must be the
	 *            same length.
	 */
public:
	HungarianAlg3(vector< vector<double> > &_costMatrix);
	~HungarianAlg3();

	vector<int> execute();
	double computeCost(vector< vector<double> > &matrix, vector<int>& assignment);

protected:
	void computeInitialFeasibleSolution();
	void executePhase(); 
	int fetchUnmatchedWorker();
	void greedyMatch();
	void initializePhase(int w);
	void match(int w, int j);
	void reduce();
	void updateLabeling(double slack);
};

#endif //_HUNGARIAN_ALG3_H_
