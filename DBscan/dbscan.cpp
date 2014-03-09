#include "dbscan.h"

DBScan::DBScan(double _eps, int _minPts) {
	eps = _eps;
	minPts = _minPts;
}

void DBScan::expandCluster(std::vector<int> neighbourList, bool *visited,
		int clusteringSetSize, double **dist, std::vector<int> & cluster, int &C) {
	// testing the neighbours
	for (int j = 0; j < neighbourList.size(); j++) {
		int x = neighbourList[j];

		// If not visited
		if (visited[x] != 1) {
			visited[x] = 1;
			std::vector<int> neighbourNeighbourList;

			// Calculating the number of neighbours
			for (int k = 0; k < clusteringSetSize; k++) {
				if (dist[std::min(x, k)][std::max(x, k)] < eps) {
					neighbourNeighbourList.push_back(k);
				}
			}

			// If it has enough neighbours it's neighbours can be checked
			if (neighbourNeighbourList.size() >= minPts) {
				for (int g = 0; g < neighbourNeighbourList.size(); g++)
					neighbourList.push_back(neighbourNeighbourList[g]);
			}
		}

		// if it is not yet labeled
		if (cluster[x] == 0)
			cluster[x] = C;
	}
}



void DBScan::run(std::vector<cv::KeyPoint> & clusteringSet, std::vector<int> & cluster) {
	int clusteringSetSize = clusteringSet.size();

	// Calculating similarity matrix
	double **dist;

	dist = new double *[clusteringSetSize];
	for (int i = 0; i < clusteringSetSize; i++)
		dist[i] = new double[clusteringSetSize];

	for (int i = 0; i < clusteringSetSize; i++)
		for (int j = i; j < clusteringSetSize; j++)
			dist[i][j] = cv::norm(clusteringSet[i].pt - clusteringSet[j].pt);

	// Preparation
	int C = 0;
	bool * visited = new bool[clusteringSetSize]();
	bool * noise = new bool[clusteringSetSize]();

	for (int i = 0; i < clusteringSetSize; i++)
		cluster.push_back(0);

	// For all points
	for (int i = 0; i < clusteringSetSize; i++) {
		if (visited[i] != true) {
			visited[i] = true;
			std::vector<int> neighbourList;

			// Finding neighbours
			for (int k = 0; k < clusteringSetSize; k++) {
				if (dist[std::min(i,k)][std::max(i,k)] < eps) {
					neighbourList.push_back(k);
				}
			}

			// If there are not enough neughbours to form a cluster
			if (neighbourList.size() < minPts)
				noise[i] = true;
			else {
				C++;

				// Test if the cluster can be expanded
				cluster[i] = C;

				expandCluster(neighbourList, visited, clusteringSetSize,
						dist, cluster, C);
			}
		}
	}

	// Clearing
	delete[] visited;
	delete[] noise;
	for (int i = 0; i < clusteringSetSize; i++)
		delete[] dist[i];
	delete[] dist;
}


