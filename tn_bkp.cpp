#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>
#include <algorithm>

#pragma pack(push, 1)
struct TrackedObject {
	bool use = false;
  uint32_t numpoints = 0;
  
  float xpos = 0.0;
  float ypos = 0.0; //TODO: need to be int?
  
  float tlx = std::numeric_limits<float>::max();
  float tly = std::numeric_limits<float>::max();
  float brx = std::numeric_limits<float>::lowest();
  float bry = std::numeric_limits<float>::lowest();

  float area = 0.0;

  uint8_t doppler_dir = 0;
  uint8_t status = 0; //0: unconfirmed, 1: confirmed, 2: deleted
  int id = 0;
  float score = 0.0;
  float max_score = 0.0;

  float heading = 0.0;
  float vel = 0.0;
  float accell = 0.0;

  float est_xpos = 0.0;
  float est_ypos = 0.0;

  float est_tlx = 0.0;
  float est_tly = 0.0;
  float est_brx = 0.0;
  float est_bry = 0.0;
	int time_birth = 0;
  int 	last_update = 0;
  uint8_t classification = 0; // 0: undefined, 1: stationary, 2: moving
  int age = 0;
};


struct point3 {
	float x, y, z;

	bool operator==(const point3& other) const {
		return x == other.x && y == other.y && z == other.z;
	}
};
#pragma pack(pop)

namespace std {
	template<>
	struct hash<point3> {
		size_t operator()(const point3& p) const {
			return hash<float>()(p.x) ^ hash<float>()(p.y) ^ hash<float>()(p.z);
		}
	};
}

float calculateDistance(const point3& p1, const point3& p2) {
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
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

void dbscan3d(std::vector<point3>& points, double epsilon, int minPts, std::vector<int>& labels) {
	// -1 is noise
	labels.assign(points.size(), 0); // Initialize labels to 0 (unvisited)
	int clusterId = 0;
	for (int i = 0; i < points.size(); ++i) {
		if (labels[i] != 0) continue; // Skip already visited points

		std::vector<int> neighbors = regionQuery(points, points[i], epsilon);
		if (neighbors.size() < minPts) {
			labels[i] = -1; // Mark as noise
		} else {
			++clusterId;
			expandCluster(points, labels, i, clusterId, epsilon, minPts);
		}
	}
}


/**
 * Creates TrackedObject instances based on the given points, labels, and unique_labels.
 * 
 * @param points The vector of point3 objects representing the points.
 * @param labels The vector of integers representing the labels of the points.
 * @param unique_labels The number of unique labels.
 * @param objects The vector of TrackedObject instances to be populated.
 */
void createObjects(std::vector<point3>& points, std::vector<int>& labels, int unique_labels, std::vector<TrackedObject>& objects, int& objectIds, int curr_time) {
	// todo handle no detections
	for (int i=1; i <= unique_labels; i++) {
		TrackedObject obj;
		float x, y, z = 0;
		float min_x, min_y, max_x, max_y = 0;
		float x_sum, y_sum = 0;
		for (int pi=0; pi < points.size(); pi++) {
			int lbl = labels[pi];
			if (i == lbl) { // evaluate this
				obj.numpoints++;
				x = points[pi].x;
				y = points[pi].y;
				//z = points[pi].z;

				// Update min and max x and y values
				min_x = std::min(obj.tlx, x);
				max_x = std::max(obj.brx, x);
				min_y = std::min(obj.tly, y);
				max_y = std::max(obj.bry, y);

				// Update the midpoint of obj
				x_sum += x;
				y_sum += y;
				obj.tlx = min_x;
				obj.tly = min_y;
				obj.brx = max_x;
				obj.bry = max_y;
			}
		}

		obj.time_birth = curr_time;
		obj.xpos = x_sum / obj.numpoints;
		obj.ypos = y_sum / obj.numpoints;
		obj.area = (obj.brx - obj.tlx) * (obj.bry - obj.tly);


		// Increment objectIds and assign it to obj.id
		// TODO: add possibility to reuse deleted ids
		objectIds++;
		obj.id = objectIds;
		obj.use = true;

		// Add object to bojects
		objects[i - 1] = obj;
	}
}

/**
 * Creates TrackedObject instances based on the given points, labels, and unique_labels.
 * 
 * @param allObjects 
 * @param newObjects 
 */
void matchObjects(std::vector<TrackedObject>& allObjects, std::vector<TrackedObject>& newObjects) {
	for (int no=0; no < newObjects.size(); no++) {
		TrackedObject newObj = newObjects[no];
		bool match = false;

		for (int ai=0; ai<allObjects.size();ai++) {
			TrackedObject obj = allObjects[ai];
			if ( (newObj.xpos < obj.brx) && (newObj.xpos > obj.tlx) && (newObj.ypos < obj.bry) && (newObj.ypos > obj.tly) ) {
				// center of new is inside old bbox!
				// TODO: improve this! and do for estimated pos instead
				match = true;
				int dt = newObj.time_birth - obj.last_update;

				obj.last_update = newObj.time_birth;
				obj.age = obj.last_update - obj.time_birth;


				float dx = newObj.xpos - obj.xpos; 
				float dy = newObj.ypos - obj.ypos;
				float dist = sqrt(dx*dx + dy*dy);
				float vel = dist / dt;
				obj.vel = (obj.vel + vel) / 2;
				float accell = vel / dt;
				obj.accell = (obj.accell + accell) / 2;
				obj.heading = atan2(dy, dx);

				// Then update all parameters
				obj.xpos = newObj.xpos;
				obj.ypos = newObj.ypos;

				obj.tlx = newObj.tlx;
				obj.tly = newObj.tly;
				obj.brx = newObj.brx;
				obj.bry = newObj.bry;
				obj.area = newObj.area;
				obj.numpoints = newObj.numpoints;
				obj.score += 1; //TODO: set as parameter in ros
				// Update status for those with score above threshold
				// TODO: add velocity calculation
			}
		}

		// if no match or no objs
		if (!match) {
			// add new object to all objects
			newObj.last_update = newObj.time_birth;
			newObj.score += 1;
			allObjects.push_back(newObj);
		}
	}
}






int main() {

	std::ifstream file("multi_data.csv");
	std::string line;

	std::vector<std::vector< point3 > > data;
	std::vector<std::vector< int > > labels;
	std::vector<std::vector< int > > gt_labels;
	std::vector<std::vector< int> > times;
				
	// Initialize new rows for each vector of vectors
	times.push_back(std::vector<int>());
	gt_labels.push_back(std::vector<int>());
	data.push_back(std::vector<point3>());
	labels.push_back(std::vector<int>());

	std::vector<TrackedObject> allObjects;
	int objectIds = 0;
	int ti = 0;
	int prev_time = 0;

	int row_i = 0;
	while (std::getline(file, line)) {
		if (row_i > 0) { // Skip header row if present
			std::stringstream ss(line);
			std::string cell;
			std::getline(ss, cell, ',');
			int time = std::stoi(cell);
			if (time != prev_time) {
				ti++;
				prev_time = time;

                // Initialize new rows for each vector of vectors
                times.push_back(std::vector<int>());
                gt_labels.push_back(std::vector<int>());
                data.push_back(std::vector<point3>());
                labels.push_back(std::vector<int>());
			}
			times[ti].push_back(time);

			std::getline(ss, cell, ',');
			gt_labels[ti].push_back(std::stoi(cell));

			point3 p;
			std::getline(ss, cell, ',');
			p.x = std::stof(cell);
			std::getline(ss, cell, ',');
			p.y = std::stof(cell);
			std::getline(ss, cell, ',');
			p.z = std::stof(cell);
			data[ti].push_back(p);
		}
		row_i++;
	}

	std::cout << "ti " << ti << std::endl;
	
	double epsilon = 0.9;
	int minPts = 5;

	for (int r=0; r<ti+1; r++) {
		std::cout << "data legnth: " << data[r].size() << std::endl;
		dbscan3d(data[r], epsilon, minPts, labels[r]);

		// Process and print label counts, comparing with ground truth
		std::unordered_map<int, int> labelCount;
		int correct = 0;
		int total = 0;
		for (int i = 0; i < labels.size(); ++i) {
			int label = labels[r][i];
			int groundTruthLabel = gt_labels[r][i];

			labelCount[label]++;

			// std::cout << "Data point " << i << ": Label = " << label << ", Ground Truth Label = " << groundTruthLabel << std::endl;
			if (label == groundTruthLabel) {
				correct++;
			}
			total++;
		}

		float match = static_cast<float>(correct) / static_cast<float>(total);
		std::cout << "Correct: " << correct << " rate: " << match << std::endl;

		std::cout << "Label Counts:" << labelCount.size() << std::endl;
		for (const auto& pair : labelCount) {
			std::cout << "Label " << pair.first << ": " << pair.second << " data points" << std::endl;
		}


		int unique_labels = 0;
		if (labelCount.size() > 1) {
			unique_labels = labelCount.size()-1;
		}

		// -------- boxing -------
		int current_time = times[r][times.size() - 1];
		std::vector<TrackedObject> newObjects(unique_labels);
		createObjects(data[r], labels[r], unique_labels, newObjects, objectIds, current_time);

		std::cout << "ids: "  << objectIds << std::endl;
		std::cout << "Objects: "  << unique_labels << std::endl;

		for (int o=0; o<newObjects.size(); o++) {
			TrackedObject obj = newObjects[o];
			std::cout << "ID: " << obj.id << " pts: " << obj.numpoints << " x: " << obj.xpos << " y: " << obj.ypos;
			std::cout << " tlx: " << obj.tlx << std::endl;
		}

		// --------- Matching ----------
		matchObjects(allObjects, newObjects);
		std::cout << "All objects: " << allObjects.size() << std::endl;
		for (const auto& obj : allObjects) {
			std::cout << "ID: " << obj.id << " Velocity: " << obj.vel << " Heading: " << obj.heading << std::endl;
		}
		std::cout << "-----------------" << std::endl;
	}

	return 0;
}
