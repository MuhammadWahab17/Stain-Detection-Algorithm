#include "HungarianAlg3.h"
#include <string.h>
/**
	* Construct an instance of the algorithm.
	* 
	* @param costMatrix
	*            the cost matrix, where matrix[i][j] holds the cost of
	*            assigning worker i to job j, for all i, j. The cost matrix
	*            must not be irregular in the sense that all rows must be the
	*            same length.
	*/
HungarianAlg3::HungarianAlg3(vector< vector<double> > &_costMatrix)
{
	int M = _costMatrix.size();
	int N = _costMatrix[0].size();
	if( M > N ) dim = M;
	else		dim = N;

	rows = _costMatrix.size();
	cols = _costMatrix[0].size();
	costMatrix = new double[dim*dim];

	for (unsigned int w = 0; w < dim; w++) {
		if (w < _costMatrix.size()) {
			if (_costMatrix[w].size() != cols) {
				cout << "Irregular cost matrix";
			}
			//costMatrix[w] = Arrays.copyOf(costMatrix[w], this.dim);
			for(int h = 0; h < dim; h++)
				costMatrix[w+h*dim] = _costMatrix[w][h];// = new double[this.dim];
		} else {
			for(int h = 0; h < dim; h++)
				costMatrix[w+h*dim] = 0.0;// = new double[this.dim];
		}
	}
	
	labelByWorker = new double[dim];
	labelByJob = new double[dim];
	minSlackWorkerByJob = new int[dim];
	minSlackValueByJob = new double[dim];
	committedWorkers = new bool[dim];
	parentWorkerByCommittedJob = new int[dim];
	matchJobByWorker = new int[dim];
	matchWorkerByJob = new int[dim];

	memset(labelByWorker, 0, dim * sizeof(double));
	memset(labelByJob, 0, dim * sizeof(double));
	memset(minSlackWorkerByJob, 0, dim * sizeof(int));
	memset(minSlackValueByJob, 0, dim * sizeof(double));
	memset(committedWorkers, 0, dim * sizeof(bool));
	memset(parentWorkerByCommittedJob, 0, dim * sizeof(int));
	memset(matchJobByWorker, -1, dim * sizeof(int));
	memset(matchWorkerByJob, -1, dim * sizeof(int));
}
HungarianAlg3::~HungarianAlg3()
{
	delete labelByWorker; labelByWorker = NULL;
	delete labelByJob; labelByJob = NULL;
	delete minSlackWorkerByJob; minSlackWorkerByJob = NULL;
	delete minSlackValueByJob; minSlackValueByJob = NULL;
	delete committedWorkers; committedWorkers = NULL;
	delete parentWorkerByCommittedJob; parentWorkerByCommittedJob = NULL;
	delete matchJobByWorker; matchJobByWorker = NULL;
	delete matchWorkerByJob; matchWorkerByJob = NULL;
}

/**
	* Compute an initial feasible solution by assigning zero labels to the
	* workers and by assigning to each job a label equal to the minimum cost
	* among its incident edges.
	*/
void HungarianAlg3::computeInitialFeasibleSolution()
{
	for (int j = 0; j < dim; j++) {
		labelByJob[j] = -1;//Double.POSITIVE_INFINITY;
	}
	for (int w = 0; w < dim; w++) {
		for (int j = 0; j < dim; j++) {
			if (costMatrix[w + j*dim] < labelByJob[j]) {
				labelByJob[j] = costMatrix[w + j*dim];
			}
		}
	}
}

/**
	* Execute the algorithm.
	* 
	* @return the minimum cost matching of workers to jobs based upon the
	*         provided cost matrix. A matching value of -1 indicates that the
	*         corresponding worker is unassigned.
	*/
vector<int> HungarianAlg3::execute()
{
	/*
		* Heuristics to improve performance: Reduce rows and columns by their
		* smallest element, compute an initial non-zero dual feasible solution
		* and create a greedy matching from workers to jobs of the cost matrix.
		*/
	reduce();
	computeInitialFeasibleSolution();
	greedyMatch();

	unsigned w = (unsigned)fetchUnmatchedWorker();
	while (w < dim) {
		initializePhase(w);
		executePhase();
		w = fetchUnmatchedWorker();
	}
	//int[] result = Arrays.copyOf(matchJobByWorker, rows);
	vector<int> result;
	for(int i = 0; i < rows; i++)
		result.push_back(matchJobByWorker[i]);

	for (w = 0; w < result.size(); w++) {
		if (result[w] >= cols) {
			result[w] = -1;
		}
	}
	return result;
}

/**
	* Execute a single phase of the algorithm. A phase of the Hungarian
	* algorithm consists of building a set of committed workers and a set of
	* committed jobs from a root unmatched worker by following alternating
	* unmatched/matched zero-slack edges. If an unmatched job is encountered,
	* then an augmenting path has been found and the matching is grown. If the
	* connected zero-slack edges have been exhausted, the labels of committed
	* workers are increased by the minimum slack among committed workers and
	* non-committed jobs to create more zero-slack edges (the labels of
	* committed jobs are simultaneously decreased by the same amount in order
	* to maintain a feasible labeling).
	* <p>
	* 
	* The runtime of a single phase of the algorithm is O(n^2), where n is the
	* dimension of the internal square cost matrix, since each edge is visited
	* at most once and since increasing the labeling is accomplished in time
	* O(n) by maintaining the minimum slack values among non-committed jobs.
	* When a phase completes, the matching will have increased in size.
	*/
void HungarianAlg3::executePhase()
{
	while (true) {
		int minSlackWorker = 0;//-1
		int minSlackJob = 0;// -1;
		double minSlackValue = 0.0;//Double.POSITIVE_INFINITY;
		for (int j = 0; j < dim; j++) {
			if (parentWorkerByCommittedJob[j] == -1) {
				if (minSlackValueByJob[j] < minSlackValue) {
					minSlackValue = minSlackValueByJob[j];
					minSlackWorker = minSlackWorkerByJob[j];
					minSlackJob = j;
				}
			}
		}
		if (minSlackValue > 0) {
			updateLabeling(minSlackValue);
		}

		parentWorkerByCommittedJob[minSlackJob] = minSlackWorker;
		if (matchWorkerByJob[minSlackJob] == -1) {
			/*
				* An augmenting path has been found.
				*/
			int committedJob = minSlackJob;
			int parentWorker = parentWorkerByCommittedJob[committedJob];
			if( parentWorker >= 0 )
			{
				while (true) {
					int temp = matchJobByWorker[parentWorker];
					match(parentWorker, committedJob);
					committedJob = temp;
					if (committedJob == -1) {
						break;
					}
					parentWorker = parentWorkerByCommittedJob[committedJob];
				}
			}
			return;
		} else {
			/*
				* Update slack values since we increased the size of the
				* committed workers set.
				*/
			int worker = matchWorkerByJob[minSlackJob];
			if( worker >= 0 )
			{
				committedWorkers[worker] = true;
				for (int j = 0; j < dim; j++) {
					if (parentWorkerByCommittedJob[j] == -1) {
						double slack = costMatrix[worker + j*dim] - labelByWorker[worker] - labelByJob[j];
						if (minSlackValueByJob[j] > slack) {
							minSlackValueByJob[j] = slack;
							minSlackWorkerByJob[j] = worker;
						}
					}
				}
			}
		}
	}
}

/**
	* 
	* @return the first unmatched worker or {@link #dim} if none.
	*/
int HungarianAlg3::fetchUnmatchedWorker()
{
	int w = 0;
	for (w = 0; w < dim; w++) {
		if (matchJobByWorker[w] == -1) {
			break;
		}
	}
	return w;
}

/**
	* Find a valid matching by greedily selecting among zero-cost matchings.
	* This is a heuristic to jump-start the augmentation algorithm.
	*/
void HungarianAlg3::greedyMatch()
{
	for (int w = 0; w < dim; w++) {
		for (int j = 0; j < dim; j++) {
			if (matchJobByWorker[w] == -1
					&& matchWorkerByJob[j] == -1
					&& costMatrix[w + j*dim] - labelByWorker[w] - labelByJob[j] == 0) {
				match(w, j);
			}
		}
	}
}

/**
	* Initialize the next phase of the algorithm by clearing the committed
	* workers and jobs sets and by initializing the slack arrays to the values
	* corresponding to the specified root worker.
	* 
	* @param w
	*            the worker at which to root the next phase.
	*/
void HungarianAlg3::initializePhase(int w)
{
	//Arrays.fill(committedWorkers, false);
	//Arrays.fill(parentWorkerByCommittedJob, -1);
	for(int i = 0; i < dim; i++)
	{
		committedWorkers[i] = false;
		parentWorkerByCommittedJob[i] = -1;
	}

	for(int i = 0; i < dim; i++)
	committedWorkers[w] = true;
	for (int j = 0; j < dim; j++) {
		minSlackValueByJob[j] = costMatrix[w + j*dim] - labelByWorker[w] - labelByJob[j];
		minSlackWorkerByJob[j] = w;
	}
}

/**
	* Helper method to record a matching between worker w and job j.
	*/
void HungarianAlg3::match(int w, int j)
{
	matchJobByWorker[w] = j;
	matchWorkerByJob[j] = w;
}

/**
	* Reduce the cost matrix by subtracting the smallest element of each row
	* from all elements of the row as well as the smallest element of each
	* column from all elements of the column. Note that an optimal assignment
	* for a reduced cost matrix is optimal for the original cost matrix.
	*/
void HungarianAlg3::reduce()
{
	for (int w = 0; w < dim; w++) {
		double min = -1;//Double.POSITIVE_INFINITY;
		for (int j = 0; j < dim; j++) {
			if (costMatrix[w+j*dim] < min) {
				min = costMatrix[w+j*dim];
			}
		}
		for (int j = 0; j < dim; j++) {
			costMatrix[w+j*dim] -= min;
		}
	}
	double* min = new double[dim];
	for (int j = 0; j < dim; j++) {
		min[j] = -1;//Double.POSITIVE_INFINITY;
	}
	for (int w = 0; w < dim; w++) {
		for (int j = 0; j < dim; j++) {
			if (costMatrix[w+j*dim] < min[j]) {
				min[j] = costMatrix[w+j*dim];
			}
		}
	}
	for (int w = 0; w < dim; w++) {
		for (int j = 0; j < dim; j++) {
			costMatrix[w+j*dim] -= min[j];
		}
	}
	delete min; min = NULL;
}

/**
	* Update labels with the specified slack by adding the slack value for
	* committed workers and by subtracting the slack value for committed jobs.
	* In addition, update the minimum slack values appropriately.
	*/
void HungarianAlg3::updateLabeling(double slack)
{
	for (int w = 0; w < dim; w++) {
		if (committedWorkers[w]) {
			labelByWorker[w] += slack;
		}
	}
	for (int j = 0; j < dim; j++) {
		if (parentWorkerByCommittedJob[j] != -1) {
			labelByJob[j] -= slack;
		} else {
			minSlackValueByJob[j] -= slack;
		}
	}
}
	
double HungarianAlg3::computeCost(vector< vector<double> > &matrix, vector<int>& assignment)
{
	double result = 0;
	//vector<int> visited;// = new HashSet<Integer>();
	for (unsigned int i = 0; i < matrix.size(); i++)
	{
		if (assignment[i] == -1) {
			continue;
		}
		//if (!visited.push_back(assignment[i])) {
		//	count << "fail";
		//}
		result += matrix[i][assignment[i]];
	}
	return result;
}
