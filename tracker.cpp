#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>

// Helper function to calculate Euclidean distance between two data points
double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2) {
	int dimensions = point1.size();
	double distance = 0.0;

	for (int i = 0; i < dimensions; i++) {
		distance += pow(point1[i] - point2[i], 2);
	}

	return sqrt(distance);
}

// Helper function to get neighbors of a data point
std::vector<int> getNeighbors(const std::vector< std::vector<double> >& data, int pointIndex, double epsilon) {
	std::vector<int> neighbors;
	int dataSize = data.size();

	for (int i = 0; i < dataSize; i++) {
		if (i == pointIndex) {
			continue;
		}

		double distance = calculateDistance(data[pointIndex], data[i]);

		if (distance <= epsilon) {
			neighbors.push_back(i);
		}
	}

	return neighbors;
}



// Helper function to expand a cluster
void expandCluster(const std::vector<std::vector<double> >& data, int pointIndex, const std::vector<int>& neighbors, int currentCluster, double epsilon, int minPts, std::vector<bool>& visited, std::vector<int>& cluster, std::vector<int>& labels) {
	cluster[pointIndex] = currentCluster;
	labels[pointIndex] = currentCluster;

	for (int i = 0; i < neighbors.size(); i++) {
		int neighborIndex = neighbors[i];

		if (!visited[neighborIndex]) {
			visited[neighborIndex] = true;

			std::vector<int> neighborNeighbors = getNeighbors(data, neighborIndex, epsilon);

			if (neighborNeighbors.size() >= minPts) {
				neighbors.insert(neighbors.end(), neighborNeighbors.begin(), neighborNeighbors.end());
			}
		}

		if (cluster[neighborIndex] == -1) {
			cluster[neighborIndex] = currentCluster;
			labels[neighborIndex] = currentCluster;
		}
	}
}


// DBSCAN clustering function
void dbscanClustering(const std::vector< std::vector< double > >& data, double epsilon, int minPts, std::vector<int>& labels) {
	int dataSize = data.size();
	std::vector<bool> visited(dataSize, false);
	std::vector<int> cluster(dataSize, -1);
	int currentCluster = 0;

	for (int i = 0; i < dataSize; i++) {
		if (visited[i]) {
			continue;
		}
		visited[i] = true;

		std::vector<int> neighbors = getNeighbors(data, i, epsilon);

		if (neighbors.size() < minPts) {
			labels[i] = -1; // Noise point
		} else {
			expandCluster(data, i, neighbors, currentCluster, epsilon, minPts, visited, cluster, labels);
			currentCluster++;
		}
	}
}

int main() {
    // Read CSV file
    std::ifstream file("data.csv");
    std::string line;
    std::vector<std::vector<double> > data;
    std::vector<int> labels;

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }

        data.push_back(row);
    }

    // Perform DBSCAN clustering
    double epsilon = 0.5; // Set your desired epsilon value
    int minPts = 5; // Set your desired minimum number of points
    dbscanClustering(data, epsilon, minPts, labels);

    // Compare the labels with the ground truth
    std::unordered_map<int, int> labelCount;
    for (int i = 0; i < labels.size(); i++) {
        int label = labels[i];
        int groundTruthLabel = data[i][0]; // Assuming the label is in the first column

        if (labelCount.find(label) == labelCount.end()) {
            labelCount[label] = 1;
        } else {
            labelCount[label]++;
        }

        std::cout << "Data point " << i << ": Label = " << label << ", Ground Truth Label = " << groundTruthLabel << std::endl;
    }

    // Print label counts
    std::cout << "Label Counts:" << std::endl;
    for (const auto& pair : labelCount) {
        std::cout << "Label " << pair.first << ": " << pair.second << " data points" << std::endl;
    }

    return 0;
}
