
#include <iostream>
#include <vector>
#include <cmath>


struct point3 {
    double x;
    double y;
    double z;

    bool operator==(const point3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

double calculateDistance(const point3& p1, const point3& p2);

std::vector<int> regionQuery(const std::vector<point3>& points, const point3& point, double epsilon);

void expandCluster(const std::vector<point3>& points, std::vector<bool>& visited, const std::vector<int>& neighbors,
                   std::vector<point3>& cluster, double epsilon, int minPts);

// std::vector<std::vector<point3>> dbscan3d(const std::vector<point3>& points, double epsilon, int minPts) {
// 	std::vector<std::vector<point3>> clusters;
// 	std::vector<bool> visited(points.size(), false);

// 	for (int i = 0; i < points.size(); ++i) {
// 		if (visited[i]) {
// 			continue;
// 		}
// 		visited[i] = true;

// 		std::vector<point3> cluster;
// 		std::vector<int> neighbors = regionQuery(points, points[i], epsilon);

// 		if (neighbors.size() < minPts) {
// 			continue;
// 		}

// 		expandCluster(points, visited, neighbors, cluster, epsilon, minPts);
// 		clusters.push_back(cluster);
// 	}

// 	return clusters;
// }

std::vector<int> dbscan3d(const std::vector<point3>& points, double epsilon, int minPts);


#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>  // for std::find


#include "radar_tracker/dbscan3d.hpp"


double calculateDistance(const point3& p1, const point3& p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	double dz = p1.z - p2.z;
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<int> regionQuery(const std::vector<point3>& points, const point3& point, double epsilon) {
	std::vector<int> neighbors;

	for (int i = 0; i < points.size(); ++i) {
		if (calculateDistance(points[i], point) <= epsilon) {
			neighbors.push_back(i);
		}
	}

	return neighbors;
}

void expandCluster(const std::vector<point3>& points, std::vector<bool>& visited, const std::vector<int>& neighbors,
                   std::vector<point3>& cluster, double epsilon, int minPts) {
    std::vector<int> modifiableNeighbors = neighbors;  // Create a modifiable copy

    for (int i = 0; i < modifiableNeighbors.size(); ++i) {
        int index = modifiableNeighbors[i];
        if (!visited[index]) {
            visited[index] = true;
            std::vector<int> newNeighbors = regionQuery(points, points[index], epsilon);
            if (newNeighbors.size() >= minPts) {
                modifiableNeighbors.insert(modifiableNeighbors.end(), newNeighbors.begin(), newNeighbors.end());
            }
        }
        if (std::find(cluster.begin(), cluster.end(), points[index]) == cluster.end()) {
            cluster.push_back(points[index]);
        }

    }
}

std::vector<int> dbscan3d(const std::vector<point3>& points, double epsilon, int minPts) {
	std::vector<int> labels(points.size(), 0);
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

	return labels;
}
