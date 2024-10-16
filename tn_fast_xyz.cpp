#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <iomanip> // Add this header for setprecision and fixed
#include <chrono>
#include <functional> // For std::greater
#include <queue> // Include the queue header file
#include <future> // For std::async and std::future
#include<cmath>

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
  int status = 0; //0: unconfirmed, 1: confirmed, 2: deleted
  int id = 0;
  int score = 0;
  int max_score = 0;

	float heading[10] = {0.0};
	float vel[10] = {0.0};
	float accell[10] = {0.0};
  int history_index = 0;

  float est_heading = 0.0;
  float est_vel = 0.0;
  float est_accell = 0.0;

  float est_xpos = 0.0;
  float est_ypos = 0.0;

  float est_tlx = 0.0;
  float est_tly = 0.0;
  float est_brx = 0.0;
  float est_bry = 0.0;
	float time_birth = 0.0;
  float 	last_update = 0;
  int classification = 0; // 0: undefined, 1: stationary, 2: moving
  float age = 0;
};


struct point3 {
	float x, y, z;
};

#pragma pack(pop)

// This remains a straightforward optimization
inline float squaredDistance(const point3& p1, const point3& p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

std::vector<int> regionQuery(const std::vector<point3>& points, const point3& point, float squaredEpsilon) {
    std::vector<int> neighbors;
	#pragma omp parallel for
    for (int i = 0; i < points.size(); ++i) {
        if (squaredDistance(points[i], point) <= squaredEpsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}


// std::vector<int> parallelRegionQuery(const std::vector<point3>& points, const point3& point, float squaredEpsilon) {
//     std::vector<std::future<bool> > futures;

//     // Launch asynchronous tasks to check each point
//     for (const auto& otherPoint : points) {
//         futures.push_back(std::async(std::launch::async, [&point, &otherPoint, squaredEpsilon]() {
//             return squaredDistance(point, otherPoint) <= squaredEpsilon;
//         }));
//     }

//     std::vector<int> neighbors;
//     for (int i = 0; i < points.size(); ++i) {
//         if (futures[i].get()) { // Wait for the result and check it
//             neighbors.push_back(i);
//         }
//     }
//     return neighbors;
// }

// std::vector<int> parallelRegionQuery(const std::vector<point3>& points, const point3& point, float squaredEpsilon) {
//     const size_t numThreads = 10;  // Total number of threads
//     std::vector<std::future<std::vector<int>>> futures;
//     std::vector<int> neighbors;
//     size_t pointsPerThread = std::ceil(points.size() / static_cast<double>(numThreads));

//     // Launch one thread for each part of the dataset
//     for (size_t start = 0; start < points.size(); start += pointsPerThread) {
//         futures.push_back(std::async(std::launch::async, [&points, &point, start, pointsPerThread, squaredEpsilon]() {
//             std::vector<int> localNeighbors;
//             // Each thread processes its allocated segment of the points
//             for (size_t i = start; i < std::min(start + pointsPerThread, points.size()); ++i) {
//                 if (squaredDistance(point, points[i]) <= squaredEpsilon) {
//                     localNeighbors.push_back(i);
//                 }
//             }
//             return localNeighbors;
//         }));
//     }

//     // Wait for all threads to complete and collect their results
//     for (auto& fut : futures) {
//         std::vector<int> localNeighbors = fut.get(); // Retrieve result from each future
//         neighbors.insert(neighbors.end(), localNeighbors.begin(), localNeighbors.end()); // Combine results into one vector
//     }

//     return neighbors;
// }


void expandCluster(const std::vector<point3>& points, std::vector<int>& labels, int index, int clusterId, float squaredEpsilon, int minPts) {
    std::vector<int> searchQueue = regionQuery(points, points[index], squaredEpsilon);

    int i = 0;
    while (i < searchQueue.size()) {
        int pointIndex = searchQueue[i++];
        if (labels[pointIndex] == 0) {
            labels[pointIndex] = clusterId;
            std::vector<int> pointNeighbors = regionQuery(points, points[pointIndex], squaredEpsilon);
            if (pointNeighbors.size() >= minPts) {
                searchQueue.insert(searchQueue.end(), pointNeighbors.begin(), pointNeighbors.end());
            }
        } else if (labels[pointIndex] == -1) {
            labels[pointIndex] = clusterId;
        }
    }
}

void dbscan3d(std::vector<point3>& points, float epsilon, int minPts, std::vector<int>& labels) {
    float squaredEpsilon = epsilon * epsilon;
    labels.assign(points.size(), 0);
    int clusterId = 0;
    for (int i = 0; i < points.size(); ++i) {
        if (labels[i] != 0) continue;

        std::vector<int> neighbors = regionQuery(points, points[i], squaredEpsilon);
        if (neighbors.size() < minPts) {
            labels[i] = -1;
        } else {
            ++clusterId;
            expandCluster(points, labels, i, clusterId, squaredEpsilon, minPts);
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
void createObjects(std::vector<point3>& points, std::vector<int>& labels, int unique_labels, std::vector<TrackedObject>& objects, int& maxObjectId, std::priority_queue<int, std::vector<int>, std::greater<int> >& deletedIds, float curr_time) {
	// todo handle no detections
	for (int i=1; i <= unique_labels; i++) {
		TrackedObject obj;
		float x, y, z = 0;
		float min_x, min_y, max_x, max_y = 0;
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
				obj.tlx = min_x;
				obj.tly = min_y;
				obj.brx = max_x;
				obj.bry = max_y;
			}
		}
		if (obj.numpoints == 0) {
			continue;
		}

		obj.time_birth = curr_time;
		obj.xpos = obj.tlx + (obj.brx - obj.tlx) / 2;
		obj.ypos = obj.tly + (obj.bry - obj.tly) / 2;
		obj.area = (obj.brx - obj.tlx) * (obj.bry - obj.tly);


		// Increment objectIds and assign it to obj.id
		// TODO: add possibility to reuse deleted ids
		if (!deletedIds.empty()) {
			obj.id = deletedIds.top();
			std::cout << "Reusing id: " << obj.id << std::endl;
			deletedIds.pop();
		} else {
			obj.id = maxObjectId++;
		}
		
		obj.use = true;

		// Add object to bojects
		objects.push_back(obj);
	}
}

/**
 * Creates TrackedObject instances based on the given points, labels, and unique_labels.
 *
 * @param allObjects
 * @param newObjects
 */
void matchObjects(std::vector<TrackedObject>& allObjects, std::vector<TrackedObject>& newObjects, std::priority_queue<int, std::vector<int>, std::greater<int> >& deletedIds) {
	for (TrackedObject& newObj : newObjects) {
		bool match = false;
		for (TrackedObject& obj : allObjects) {
			if ( (newObj.xpos < obj.brx) && (newObj.xpos > obj.tlx) && (newObj.ypos < obj.bry) && (newObj.ypos > obj.tly) ) {
			// float pos_dist = sqrt(pow(newObj.xpos - obj.est_xpos, 2) + pow(newObj.ypos - obj.est_ypos, 2));
			// if ( pos_dist < 5.0 ) { // if the estimated new position and the current position is less than 3m
				
				// std::cout << "matched old: " << obj.id << ", new: " << newObj.id << std::endl;
				// center of new is inside old bbox!
				// TODO: improve this! and do for estimated pos instead
				match = true;
				float dt = newObj.time_birth - obj.last_update;

				obj.last_update = newObj.time_birth;
				obj.age = obj.last_update - obj.time_birth;


				float dx = newObj.xpos - obj.xpos;
				float dy = newObj.ypos - obj.ypos;
				float dist = sqrt(dx*dx + dy*dy);
				float vel = dist / dt;


				// calculate est vel and heading
				obj.vel[obj.history_index] = vel;
				obj.heading[obj.history_index] = atan2(dy, dx);
				obj.accell[obj.history_index] = vel / dt;

				float sum_head = 0.0;
				float sum_vel = 0.0;
				float sum_acc = 0.0;

				for (int i = 0; i < 10; i++) {
					sum_vel += obj.vel[i];
					sum_head += obj.heading[i];
					sum_acc += obj.accell[i];
				}
				obj.est_vel = sum_vel / 10;
				obj.est_heading = sum_head / 10;
				obj.est_accell = sum_acc / 10;


				obj.history_index = (obj.history_index + 1) % 10;

				// obj.heading = atan2(dy, dx) * 180 / M_PI;
				// obj.heading = fmod(obj.heading + 90, 360); // Adjust rotation clockwise
				// if (obj.heading < 0) {
				// 	obj.heading += 360; // Ensure positive angle
				// }

				obj.est_xpos = newObj.xpos + dx; // assume constant velocity
				obj.est_ypos = newObj.ypos + dy;


				// Then update all parameters
				obj.xpos = newObj.xpos;
				obj.ypos = newObj.ypos;

				obj.tlx = newObj.tlx;
				obj.tly = newObj.tly;
				obj.brx = newObj.brx;
				obj.bry = newObj.bry;
				obj.area = newObj.area;
				obj.numpoints = newObj.numpoints;
				obj.score += 10; //TODO: set as parameter in ros

				// Update status for those with score above threshold
				// TODO: add velocity history
				obj.max_score = std::max(obj.max_score, obj.score-5);
				// TODO: delete by old age!
				break;
			}
		}

		// if no match or no objs
		if (!match) {
			// std::cout << "added obj: " << newObj.id << std::endl;
			// add new object to all objects
			newObj.last_update = newObj.time_birth;
			newObj.score += 10;
			allObjects.push_back(newObj);
		}
	}


	std::vector<int> to_delete;
	int current_id = 0;

	for (TrackedObject& obj : allObjects) {

		// Do for all
		obj.score -= 5;

		if ( ((obj.max_score - obj.score) > 5) || (obj.score < 0) ) { // ((obj.max_score - obj.score) > 5) ||
			obj.status = 2; // deleted
			to_delete.push_back(current_id);
			// std::cout << "Deleting object id: " << obj.id << " score: " << obj.score << std::endl;
		}

		// Update status for those with score above threshold
		if (obj.score > 15) { //TODO: set as parameter in ros
			obj.status = 1; // confirmed
		}

		// update classification
		if (obj.est_vel > 0.51444) { // 1 knot
			obj.classification = 2; // moving
		} else {
			obj.classification = 1; // stationary
		}


		current_id++;
	}

	for (int i=0; i < to_delete.size(); i++) {
		deletedIds.push(allObjects[to_delete[i]].id);
		allObjects.erase(allObjects.begin() + to_delete[i]);
	}


}


void printBox(const std::string& message) {
    std::string border(35, '-');
    std::string padding(35, ' ');

    std::cout << "/" << border << "\\" << std::endl;
    std::cout << "|" << padding << "|" << std::endl;
    std::cout << "|  " << message << "|" << std::endl;
    std::cout << "|" << padding << "|" << std::endl;
    std::cout << "\\" << border << "/" << std::endl;
}

int main(int argc, char* argv[]) {
    std::string wakeupMessage = "Starting MARV RADAR-Tracker v0.2 ";
    printBox(wakeupMessage);

    if (argc < 2) {
        std::cout << "Usage: ./program_name <input_csv_file>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::ifstream file(filename);
    if (!file) {
        std::cout << "Failed to open file: " << filename << std::endl;
        return 1;
    }

	// std::ifstream file("multi_data.csv");
	// Here we should load the input csv filename from argv1


	//std::ifstream file("log_1.csv");
	std::ofstream out_file("objects.csv");
	out_file << "time,id,score,status,classification,xpos,ypos,age,est_velocity,est_heading,tlx,tly,brx,bry" << std::endl;
	std::string line;

	std::vector<std::vector< point3 > > data;
	std::vector<std::vector< int > > labels;
	std::vector<std::vector< float> > times;

	// Initialize new rows for each vector of vectors
	times.push_back(std::vector<float>());
	data.push_back(std::vector<point3>());
	labels.push_back(std::vector<int>());

	std::vector<TrackedObject> allObjects;
	std::priority_queue<int, std::vector<int>, std::greater<int>> deletedIds;
	int maxObjectId = 0;
	
	int ti = 0;
	float prev_time = 0;

	auto syst_t0 = std::chrono::system_clock::now();

	int row_i = 0;
	while (std::getline(file, line)) {
		if (row_i > 0) { // Skip header row if present
			std::stringstream ss(line);
			std::string cell;
			std::getline(ss, cell, ',');
			float time = std::stof(cell);

			if ((time - prev_time)> 0.1) {
				ti++;
				prev_time = time;

				// Initialize new rows for each vector of vectors
				times.push_back(std::vector<float>());
				data.push_back(std::vector<point3>());
				labels.push_back(std::vector<int>());
			}
			times[ti].push_back(time);

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

	// std::cout << times[0][0] << " " << data[0][0].x << " " << data[0][0].y << " " << data[0][0].z << std::endl;

	auto syst_t1 = std::chrono::system_clock::now();

	auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(syst_t1 - syst_t0);
	std::cout << "CSV Load time: " << std::fixed << std::setprecision(4) << duration.count() << " seconds" << std::endl;
	

	double epsilon = 0.9; //5
	int minPts = 16; //15
	// todo: configure these

	for (int r=0; r<ti+1; r++) {
		// std::cout << "data length: " << data[r].size() << std::endl;
		if (data[r].size() < 1) {
			continue;
		}

		auto syst_cstart = std::chrono::system_clock::now();

		dbscan3d(data[r], epsilon, minPts, labels[r]);

		auto syst_cend = std::chrono::system_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(syst_cend - syst_cstart);
		std::cout << "CT: " << std::fixed << std::setprecision(4) << duration.count();

		// Process and print label counts, comparing with ground truth
		std::unordered_map<int, int> labelCount;
		int total = 0;
		for (int i = 0; i < labels[r].size(); ++i) {
			int label = labels[r][i];
			labelCount[label]++;
			total++;
		}

		// std::cout << "Label Counts:" << labelCount.size() << std::endl;
		// for (const auto& pair : labelCount) {
		// 	std::cout << "Label " << pair.first << ": " << pair.second << " data points" << std::endl;
		// }


		int unique_labels = 0;
		if (labelCount.size() > 1) {
			unique_labels = labelCount.size();
		}

		// -------- boxing -------
		float current_time = times[r][times.size() - 1];
		// std::cout << "Message Time: " << current_time << std::endl;
		std::vector<TrackedObject> newObjects;
		createObjects(data[r], labels[r], unique_labels, newObjects, maxObjectId, deletedIds, current_time);

		// std::cout << "ids: "  << objectIds << std::endl;
		// std::cout << "Objects: "  << unique_labels << std::endl;

		// for (int o=0; o<newObjects.size(); o++) {
		// 	TrackedObject obj = newObjects[o];
		// 	std::cout << "ID: " << obj.id << " pts: " << obj.numpoints << " x: " << obj.xpos << " y: " << obj.ypos;
		// 	std::cout << " tlx: " << obj.tlx << std::endl;
		// }

		// --------- Matching ----------
		matchObjects(allObjects, newObjects, deletedIds);

		auto syst_mend = std::chrono::system_clock::now();
		auto duration_2 = std::chrono::duration_cast<std::chrono::duration<double>>(syst_mend - syst_cend);
		std::cout << ", MT: " << std::fixed << std::setprecision(4) << duration_2.count() << ", T: " << std::setprecision(4) << current_time << std::endl;

		// std::cout << "All objects: " << allObjects.size() << std::endl;
		// for (const auto& ao : allObjects) {
		// 	std::cout << "ID: " << ao.id << " score: " << ao.score << " Velocity: " << ao.est_vel << " Heading: " << ao.est_heading << std::endl;
		// }
		// std::cout << "-----------------" << std::endl;

		// TODO: reuse ids

		// "time,id,score,status,classification,xpos,ypos,age,est_velocity,est_heading,tlx,tly,brx,bry"
		if (out_file.is_open()) {
			for (const auto& ao : allObjects) {
				out_file << current_time << ","; 	//time
				out_file << ao.id << ",";			//id
				out_file << ao.score << ",";		//score
				out_file << ao.status << ",";		//status
				out_file << ao.classification << ",";	//classification
				out_file << std::fixed;								//
				out_file << std::setprecision(5) << ao.xpos << ",";	//xpos
				out_file << std::setprecision(5) << ao.ypos << ",";	//ypos
				out_file << std::setprecision(5) << ao.age << ",";	//age
				out_file << std::setprecision(5) << ao.est_vel << ",";	//est_velocity
				out_file << std::setprecision(5) << ao.est_heading << ",";	//est_heading
				out_file << std::setprecision(5) << ao.tlx << ",";			//tlx
				out_file << std::setprecision(5) << ao.tly << ",";			//tly
				out_file << std::setprecision(5) << ao.brx << ",";			//brx
				out_file << std::setprecision(5) << ao.bry << std::endl;	//bry
				// out_file << current_time << "," << ao.id << "," << ao.score << "," << ao.est_vel << "," << ao.est_heading << "\n";
			}
		}
	}
	out_file.close();
	return 0;
}
