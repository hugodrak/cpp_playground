#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>


struct point3 {
    double x;
    double y;
    double z;

    bool operator==(const point3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

namespace std {
template<>
struct hash<point3> {
    size_t operator()(const point3& p) const {
        return hash<double>()(p.x) ^ hash<double>()(p.y) ^ hash<double>()(p.z);
    }
};
}

double calculateDistance(const point3& p1, const point3& p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	double dz = p1.z - p2.z;
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<int> regionQuery(const std::vector<point3>& points, const point3& point, double epsilon) {
	std::vector<int> neighbors;
	double dist = 0.0;
	for (int i = 0; i < points.size(); ++i) {
		dist = calculateDistance(points[i], point);
		std::cout << dist << std::endl;
		if (dist <= epsilon) {
			neighbors.push_back(i);
		}
	}

	return neighbors;
}

void expandCluster(const std::vector<point3>& points, std::vector<int>& labels, int index, int clusterId, double epsilon, int minPts) {
    std::vector<int> searchQueue = regionQuery(points, points[index], epsilon);
    int i = 0;
    while (i < searchQueue.size()) {
        int pointIndex = searchQueue[i++];
        if (labels[pointIndex] == 0) { // Point not yet visited
            labels[pointIndex] = clusterId;
            std::vector<int> pointNeighbors = regionQuery(points, points[pointIndex], epsilon);
            if (pointNeighbors.size() >= minPts) {
                searchQueue.insert(searchQueue.end(), pointNeighbors.begin(), pointNeighbors.end());
            }
        } else if (labels[pointIndex] == -1) { // Point was labeled as noise
            labels[pointIndex] = clusterId;
        }
    }
}


void dbscan3d(const std::vector<point3>& points, double epsilon, int minPts, std::vector<int>& labels) {
	//std::vector<int> labels(points.size(), 0);
	int clusterId = 0;

	for (int i = 0; i < points.size(); ++i) {
		if (labels[i] != 0) {
			continue;
		}

		std::vector<int> neighbors = regionQuery(points, points[i], epsilon);

		if (neighbors.size() < minPts) {
			labels[i] = -1; // Noise point
			continue;
		}

		++clusterId;
		labels[i] = clusterId;

		for (int j = 0; j < neighbors.size(); ++j) {
			int index = neighbors[j];
			if (labels[index] == -1) {
				labels[index] = clusterId;
			}
			if (labels[index] != 0) {
				continue;
			}
			labels[index] = clusterId;

			std::vector<int> newNeighbors = regionQuery(points, points[index], epsilon);
			if (newNeighbors.size() >= minPts) {
				neighbors.insert(neighbors.end(), newNeighbors.begin(), newNeighbors.end());
			}
		}
	}
}



int main() {
    // Read CSV file
    std::ifstream file("data.csv");
    std::string line;
	std::vector<point3> data;
    std::vector<int> labels;
    std::vector<int> gt_labels;
	int row_i = 0;

    while (std::getline(file, line)) {
		if (row_i > 0) {
			std::stringstream ss(line);
			std::string cell;
			std::getline(ss, cell, ',');
			gt_labels.push_back(std::stoi(cell));

			point3 p;
			std::getline(ss, cell, ',');
			p.x = std::stod(cell);
			std::getline(ss, cell, ',');
			p.y = std::stod(cell);
			std::getline(ss, cell, ',');
			p.z = std::stod(cell);
			data.push_back(p);
		}
		row_i++;
    }


    // Perform DBSCAN clustering
    double epsilon = 0.5; // Set your desired epsilon value
    int minPts = 5; // Set your desired minimum number of points
    dbscan3d(data, epsilon, minPts, labels);
	std::cout << "Labels: " << labels.size() << "pts: " << data.size() << std::endl;
    // Compare the labels with the ground truth
    std::unordered_map<int, int> labelCount;
    for (int i = 0; i < labels.size(); i++) {
        int label = labels[i];
        int groundTruthLabel = gt_labels[i]; // Assuming the label is in the first column

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
