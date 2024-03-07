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
#include <boost/functional/hash.hpp>
#include <iostream>
#include <unordered_map>

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

struct ClusterObject {
	int32_t cluster_id;
	int32_t matched_id;
	int32_t x_idx;
	int32_t y_idx;
	bool right_heavy;
	bool top_heavy;
	bool left_heavy;
	bool bottom_heavy;
	double box_centre_x;
	double box_centre_y;
	int32_t point_count;
	double max_x;
	double min_x;
	double max_y;
	double min_y;
	double mass_center_x;
	double mass_center_y;
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
    for (int i = 0; i < points.size(); ++i) {
        if (squaredDistance(points[i], point) <= squaredEpsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

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


void createInitialClusters(const std::vector<point3>& points, float x_min, float y_min, float x_size, float y_size, std::vector<std::vector<int>>& initial_clusters) {
	std::unordered_map<std::pair<int, int>, std::vector<int>, boost::hash<std::pair<int, int>>> cluster_map;

	for (int i = 0; i < points.size(); ++i) {
		int x_idx = static_cast<int>((points[i].x - x_min) / x_size);
		int y_idx = static_cast<int>((points[i].y - y_min) / y_size);
		cluster_map[std::make_pair(x_idx, y_idx)].push_back(i);
	}

	initial_clusters.reserve(cluster_map.size());
	for (const auto& entry : cluster_map) {
		initial_clusters.push_back(entry.second);
	}
}


void merge_clusters(std::vector<Cluster>& clusters, int current_id, double D, int matchid) {
    Cluster& current_cluster = clusters[current_id];
    current_cluster.setNotCheck(true);

    // Check up
    if (!current_cluster.isBottomHeavy()) {
        Cluster* up = current_cluster.getUpNeighbor(clusters);
        if (up != nullptr && !up->isNotCheck()) {
            if (up->getMatchedId() == -1) {
                if (std::abs(current_cluster.getMaxY() - up->getMinY()) < D) {
                    if (!up->isTopHeavy()) {
                        merge_clusters(clusters, up->getClusterId(), D, matchid);
                        up->setMatchedId(matchid);
                    }
                }
            }
        }
    }

    // Check down
    if (!current_cluster.isTopHeavy()) {
        Cluster* down = current_cluster.getDownNeighbor(clusters);
        if (down != nullptr && !down->isNotCheck()) {
            if (down->getMatchedId() == -1) {
                if (std::abs(current_cluster.getMinY() - down->getMaxY()) < D) {
                    if (!down->isBottomHeavy()) {
                        merge_clusters(clusters, down->getClusterId(), D, matchid);
                        down->setMatchedId(matchid);
                    }
                }
            }
        }
    }

    // Check left
    if (!current_cluster.isRightHeavy()) {
        Cluster* left = current_cluster.getLeftNeighbor(clusters);
        if (left != nullptr && !left->isNotCheck()) {
            if (left->getMatchedId() == -1) {
                if (std::abs(current_cluster.getMinX() - left->getMaxX()) < D) {
                    if (!left->isLeftHeavy()) {
                        merge_clusters(clusters, left->getClusterId(), D, matchid);
                        left->setMatchedId(matchid);
                    }
                }
            }
        }
    }

    // Check right
    if (!current_cluster.isLeftHeavy()) {
        Cluster* right = current_cluster.getRightNeighbor(clusters);
        if (right != nullptr && !right->isNotCheck()) {
            if (right->getMatchedId() == -1) {
                if (std::abs(current_cluster.getMaxX() - right->getMinX()) < D) {
                    if (!right->isRightHeavy()) {
                        merge_clusters(clusters, right->getClusterId(), D, matchid);
                        right->setMatchedId(matchid);
                    }
                }
            }
        }
    }
}




/**
 * Performs grid-based density clustering on a set of 3D points.
 *
 * @param points The vector of 3D points to be clustered.
 * @param labels The vector of labels assigned to each point after clustering.
 * @param n The number of grid cells in each dimension.
 * @param r The range of the grid in each dimension.
 * @param c The center box size ratio for setting heaviness.
 * @param d The max separation between clusters.
 * @param minPts The minimum number of points required to form a cluster.
 * @param edge_limit The limit for points to be considered as edge points.
 */
void gridScanHD(std::vector<point3>& points, std::vector<int>& labels, int n, int r, float c, float d, int minPts, float edge_limit) {
	float x_size = (r * 2) / n;
	float y_size = (r * 2) / n;
	labels.assign(points.size(), 0);
	int clusterId = 0;

	std::vector<std::vector<int>>& initial_clusters;
	std::vector<ClusterObject> clusters;
	createInitialClusters(points, -r, -r, x_size, y_size, initial_clusters);

	// Assign cluster IDs and calculate centers of mass
	for (int cluster_id = 0; cluster_id < initial_clusters.size(); ++cluster_id) {
		std::vector<int>& cluster_indices = initial_clusters[cluster_id];
		// print("cluster id:", cluster_id, "contains", len(cluster_indices), "points")
		if (cluster_indices.size() < minPts) {
			for (int i = 0; i < cluster_indices.size(); ++i) {
				labels[cluster_indices[i]] = -1; // Noise
			}
			continue;
		}
		std::vector<point3> cluster_points;
		for (int i = 0; i < cluster_indices.size(); ++i) {
			cluster_points.push_back(points[cluster_indices[i]]);
		}

		int x_idx = static_cast<int>((cluster_points[0].x - (-r)) / x_size);
		int y_idx = static_cast<int>((cluster_points[0].y - (-r)) / y_size);
		double mass_center_x = 0.0;
		double mass_center_y = 0.0;
		int point_count = cluster_points.size();

		for (int i = 0; i < cluster_points.size(); ++i) {
			mass_center_x += cluster_points[i].x;
			mass_center_y += cluster_points[i].y;
		}
		mass_center_x /= cluster_points.size();
		mass_center_y /= cluster_points.size();

		double box_centre_x = -r + (x_idx + 0.5) * x_size;
		double box_centre_y = -r + (y_idx + 0.5) * y_size;

		double max_x = cluster_points[0].x;
		double min_x = cluster_points[0].x;
		double max_y = cluster_points[0].y;
		double min_y = cluster_points[0].y;

		for (int i = 1; i < cluster_points.size(); ++i) {
			if (cluster_points[i].x > max_x) {
				max_x = cluster_points[i].x;
			}
			if (cluster_points[i].x < min_x) {
				min_x = cluster_points[i].x;
			}
			if (cluster_points[i].y > max_y) {
				max_y = cluster_points[i].y;
			}
			if (cluster_points[i].y < min_y) {
				min_y = cluster_points[i].y;
			}
		}

		// do calculations to see if there exist points in the borders
		bool points_right = false;
		bool points_left = false;
		bool points_top = false;
		bool points_bottom = false;

		for (int i = 0; i < cluster_points.size(); ++i) {
			if (cluster_points[i].x > (box_centre_x + x_size / 2 * edge_limit)) {
				points_right = true;
			}
			if (cluster_points[i].x < (box_centre_x - x_size / 2 * edge_limit)) {
				points_left = true;
			}
			if (cluster_points[i].y > (box_centre_y + y_size / 2 * edge_limit)) {
				points_top = true;
			}
			if (cluster_points[i].y < (box_centre_y - y_size / 2 * edge_limit)) {
				points_bottom = true;
			}
		}

		// add heaviness bool
		bool right_heavy = (mass_center_x > box_centre_x + x_size * c) && !points_left; // we add a center box to the heaviness
		bool left_heavy = (mass_center_x < box_centre_x - x_size * c) && !points_right;
		bool top_heavy = (mass_center_y > box_centre_y + y_size * c) && !points_bottom;
		bool bottom_heavy = (mass_center_y < box_centre_y - y_size * c) && !points_top;

		ClusterObject new_clust;
		new_clust.cluster_id = cluster_id;
		new_clust.matched_id = -1; // Initialize with -1
		new_clust.x_idx = x_idx;
		new_clust.y_idx = y_idx;
		new_clust.right_heavy = right_heavy;
		new_clust.top_heavy = top_heavy;
		new_clust.left_heavy = left_heavy;
		new_clust.bottom_heavy = bottom_heavy;
		new_clust.box_centre_x = box_centre_x;
		new_clust.box_centre_y = box_centre_y;
		new_clust.point_count = point_count;
		new_clust.max_x = max_x;
		new_clust.min_x = min_x;
		new_clust.max_y = max_y;
		new_clust.min_y = min_y;
		new_clust.mass_center_x = mass_center_x;
		new_clust.mass_center_y = mass_center_y;

		clusters.push_back(new_clust);
		labels[cluster_indices[0]] = cluster_id;
	}

	// Merge clusters based on centers of mass closer than D apart
	std::vector<int> notcheck;
	for (int index = 0; index < clusters.size(); ++index) {
		ClusterObject current_box = clusters[index];
		int current_id = current_box.cluster_id;
		// Merge clusters based on centers of mass closer than D apart
		if (current_box.matched_id == -1) {
			clusters = mergeClusters(clusters, current_id, d, current_id);
		}

		if (current_box.matched_id == -1) {
			for (int i = 0; i < clusters.size(); ++i) {
				if (clusters[i].cluster_id == current_id) {
					clusters[i].matched_id = current_id;
				}
			}
		}
	}

	std::vector<int> unique_merged_ids;
	for (int i = 0; i < clusters.size(); ++i) {
		if (std::find(unique_merged_ids.begin(), unique_merged_ids.end(), clusters[i].matched_id) == unique_merged_ids.end()) {
			unique_merged_ids.push_back(clusters[i].matched_id);
		}
	}

	// Plot 2: Grid-based Clustering with Merged Clusters
	std::unordered_map<std::string, std::string> dtypes = {
		{"id", "int32"},
		{"mass_center_x", "float64"},
		{"mass_center_y", "float64"},
		{"max_x", "float64"},
		{"min_x", "float64"},
		{"max_y", "float64"},
		{"min_y", "float64"},
		{"box_centre_x", "float64"},
		{"box_centre_y", "float64"},
		{"point_count", "int32"},
		{"box_count", "int32"}
	};

	std::unordered_map<std::string, std::vector<std::string>> mc_df;
	for (const auto& entry : dtypes) {
		mc_df[entry.first] = std::vector<std::string>();
	}

	int new_id = 0;
	for (int i = 0; i < unique_merged_ids.size(); ++i) {
		int id = unique_merged_ids[i];
		std::vector<ClusterObject> new_clusters;
		for (int j = 0; j < clusters.size(); ++j) {
			if (clusters[j].matched_id == id) {
				new_clusters.push_back(clusters[j]);
			}
		}
		int box_count = new_clusters.size();
		int total_mass = 0;
		double centre_x = 0.0;
		double centre_y = 0.0;
		double max_x = new_clusters[0].max_x;
		double min_x = new_clusters[0].min_x;
		double max_y = new_clusters[0].max_y;
		double min_y = new_clusters[0].min_y;
		double box_centre_x = 0.0;
		double box_centre_y = 0.0;

		for (int j = 0; j < new_clusters.size(); ++j) {
			total_mass += new_clusters[j].point_count;
			centre_x += new_clusters[j].mass_center_x * new_clusters[j].point_count;
			centre_y += new_clusters[j].mass_center_y * new_clusters[j].point_count;
			if (new_clusters[j].max_x > max_x) {
				max_x = new_clusters[j].max_x;
			}
			if (new_clusters[j].min_x < min_x) {
				min_x = new_clusters[j].min_x;
			}
			if (new_clusters[j].max_y > max_y) {
				max_y = new_clusters[j].max_y;
			}
			if (new_clusters[j].min_y < min_y) {
				min_y = new_clusters[j].min_y;
			}
		}

		if (total_mass < mp) {
			continue;
		}

		centre_x /= total_mass;
		centre_y /= total_mass;
		box_centre_x = (max_x + min_x) / 2;
		box_centre_y = (max_y + min_y) / 2;

		mc_df["id"].push_back(std::to_string(new_id));
		mc_df["mass_center_x"].push_back(std::to_string(centre_x));
		mc_df["mass_center_y"].push_back(std::to_string(centre_y));
		mc_df["max_x"].push_back(std::to_string(max_x));
		mc_df["min_x"].push_back(std::to_string(min_x));
		mc_df["max_y"].push_back(std::to_string(max_y));
		mc_df["min_y"].push_back(std::to_string(min_y));
		mc_df["box_centre_x"].push_back(std::to_string(box_centre_x));
		mc_df["box_centre_y"].push_back(std::to_string(box_centre_y));
		mc_df["point_count"].push_back(std::to_string(total_mass));
		mc_df["box_count"].push_back(std::to_string(box_count));

		new_id++;
	}


}

	// for (int i = 0; i < initial_clusters.size(); ++i) {
	// 	std::vector<int>& cluster = initial_clusters[i];
	// 	if (cluster.size() >= minPts) {
	// 		clusterId++;
	// 		for (int j = 0; j < cluster.size(); ++j) {
	// 			labels[cluster[j]] = clusterId;
	// 		}
	// 	}
	// }



// }


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
