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
#include <map>
#include <utility> // For std::pair

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
	int cluster_id = -1;
	int matched_id = -1;
	int x_idx = 0;
	int y_idx = 0;
	bool right_heavy = 0;
	bool top_heavy = 0;
	bool left_heavy = 0;
	bool bottom_heavy = 0;
	float box_centre_x = 0.0;
	float box_centre_y = 0.0;
	int point_count = 0;
	float max_x = 0.0;
	float min_x = 0.0;
	float max_y = 0.0;
	float min_y = 0.0;
	float mass_center_x = 0.0;
	float mass_center_y = 0.0;
	bool check = true;
};



struct MergedClusterObject { // TODO: check if float or double
	int id = 0;
	float mass_center_x = 0.0;
	float mass_center_y = 0.0;
	float max_x = 0.0;
	float min_x = 0.0;
	float max_y = 0.0;
	float min_y = 0.0;
	float box_centre_x = 0.0;
	float box_centre_y = 0.0;
	int point_count = 0;
	int box_count = 0;
	bool remove = false;
};


struct point3 {
	float x, y, z;
};

#pragma pack(pop)

// void createInitialClusters(const std::vector<point3>& points, float x_min, float y_min, float x_size, float y_size, std::vector<std::vector<int>>& initial_clusters) {
// 	std::unordered_map< std::pair<int, int>, std::vector<int>, boost::hash< std::pair<int, int> > > cluster_map;

// 	for (std::vector<point3>::size_type i = 0; i < points.size(); ++i) {
// 		int x_idx = static_cast<int>((points[i].x - x_min) / x_size);
// 		int y_idx = static_cast<int>((points[i].y - y_min) / y_size);
// 		cluster_map[std::make_pair(x_idx, y_idx)].push_back(i);
// 	}

// 	initial_clusters.reserve(cluster_map.size());
// 	for (const auto& entry : cluster_map) {
// 		initial_clusters.push_back(entry.second);
// 	}
// }

void createInitialClusters(std::vector<point3>& points, int r, float box_size, std::vector<std::vector<int>>& initial_clusters) {
    // A map to hold the clusters with their grid indices as keys
    std::map<std::pair<int, int>, std::vector<int>> clusters;
	// TODO: figure out how this works

    // Assign grid indices to each point
    for (std::vector<point3>::size_type i = 0; i < points.size(); ++i) {
       	int x_idx = static_cast<int>((points[i].x + r) / box_size);
        int y_idx = static_cast<int>((r - points[i].y) / box_size);

        // Group points by their grid indices
        clusters[std::make_pair(x_idx, y_idx)].push_back(i); // i is the index of the point in the original vector
    }

    // If you need to use the clusters similarly to how they're used in Python,
    // you can iterate over the map here. For example, to mimic the 'tolist()' part:
    for (const auto& pair : clusters) {
        initial_clusters.push_back(pair.second); // 'pair.second' is the vector<int> of indices
		// std::cout << pair.first.first << " " << pair.first.second << std::endl;
		
		// for (const auto& value : pair.second) {
			
		// 	 std::cout << "P: " << value << " " << points[value].x << " " << points[value].y << std::endl;
			
		// }
		// std::cout << std::endl;
		
    }

    // Now 'initial_clusters' holds the grouped indices as in your Python code
    // Each element of 'initial_clusters' is a vector<int> representing a cluster
}

// TODO: change current id to uint_32t
void mergeClusters(std::vector<ClusterObject>& clusters, int current_id, double D, int matchid, int n, std::vector<std::vector<int> >& cluster_grid) {
    ClusterObject& current_cluster = clusters[current_id];
	// std::cout << "current_id: " << current_id << " id: (" << current_cluster.x_idx << ", " << current_cluster.y_idx << ")" << std::endl;
	// std::cout << "c_x: " << current_cluster.x_idx << " c_y: " << current_cluster.y_idx << std::endl;
    current_cluster.check = false;
	int current_x = current_cluster.x_idx;
	int current_y = current_cluster.y_idx;
	// std::cout << "bh: " << current_cluster.bottom_heavy << " th: " << current_cluster.top_heavy << " rh: " << current_cluster.right_heavy << " lh: " << current_cluster.left_heavy << std::endl;
    // Check up
    if (!current_cluster.bottom_heavy && current_y > 0) {
		int up_id = cluster_grid[current_y-1][current_x]; // TODO: check if -1 exists
		if (up_id >= 0) {
			// std::cout << "up_id: " << up_id << std::endl;
			// TODO: check if +1 exists
			ClusterObject& up = clusters[up_id];
			// std::cout << "up_x: " << up.x_idx << " up_y: " << up.y_idx << std::endl;
			// Cluster* up = current_cluster.getUpNeighbor(clusters);
			if (up.check) {
				if (up.matched_id == -1) {
					if (std::abs(current_cluster.max_y - up.min_y) < D) {
						if (!up.top_heavy) {
							// std::cout << "Looking UP" << std::endl;
							mergeClusters(clusters, up_id, D, matchid, n, cluster_grid);
							up.matched_id = matchid;
							// std::cout << "current id: " << current_id << " (" <<current_cluster.x_idx << ", " << current_cluster.y_idx << ") matched UP: " << up_id << " " << up.x_idx << ", " << up.y_idx << std::endl;
						}
					}
				}
			}
		}
    }

    // Check down, working
    if (!current_cluster.top_heavy && current_y < n-1) {
		int down_id = cluster_grid[current_y+1][current_x];
		if (down_id >= 0) {
			ClusterObject& down = clusters[down_id];
			if (down.check) {
				if (down.matched_id == -1) {
					// std::cout << "ys c, down: (" <<  current_cluster.min_y << ", " << down.max_y << ")" << std::endl;
					if (std::abs(current_cluster.min_y - down.max_y) < D) {
						if (!down.bottom_heavy) {
							// std::cout << "Looking DOWN" << std::endl;
							mergeClusters(clusters, down_id, D, matchid, n, cluster_grid);
							down.matched_id = matchid;
						}
					}
				}
			}
		}
    }

    // Check left
    if (!current_cluster.right_heavy && current_x > 0) {
		int left_id = cluster_grid[current_y][current_x-1];
		if (left_id >= 0) {
			ClusterObject& left = clusters[left_id];
			if (left.check) {
				if (left.matched_id == -1) {
					// std::cout << "Current:" << current_cluster.x_idx << ", " << current_cluster.y_idx << " UP: " << left.x_idx << ", " << left.y_idx << std::endl;
					// std::cout << "dist" << std::abs(current_cluster.min_x - left.max_x) << " D: " << D << std::endl;
					if (std::abs(current_cluster.min_x - left.max_x) < D) {
						if (!left.left_heavy) {
							// std::cout << "Looking LEFT" << std::endl;
							mergeClusters(clusters, left_id, D, matchid, n, cluster_grid);
							left.matched_id = matchid;
						}
					}
				}
			}
		}
    }

    // Check right
    if (!current_cluster.left_heavy && current_x < n-1) {
		int right_id = cluster_grid[current_y][current_x+1];
		if (right_id >= 0) {
			ClusterObject& right = clusters[right_id];
			if (right.check) {
				if (right.matched_id == -1) {
					// std::cout << "xs c, right: (" <<  current_cluster.max_x << ", " << right.min_x << ") D: " << std::abs(current_cluster.max_x - right.min_x) << std::endl;
					if (std::abs(current_cluster.max_x - right.min_x) < D) {
						if (!right.right_heavy) {
							// std::cout << "Looking RIGHT" << std::endl;
							mergeClusters(clusters, right_id, D, matchid, n, cluster_grid);
							right.matched_id = matchid;
						}
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
void gridScanHD(std::vector<point3>& points, std::vector<MergedClusterObject>& mergedClusters, int n, float r, float c, float d, int minPts, float edge_limit) {
	float box_size = (r * 2) / n;
	//labels.assign(points.size(), 0);
	int clusterId = 0;

	std::vector<std::vector<int>> initial_clusters;
	std::vector<ClusterObject> clusters;
	std::vector<std::vector<int>> cluster_grid(n, std::vector<int>(n, -1));
	
	
	createInitialClusters(points, r, box_size, initial_clusters);
	// size_t minPts_size = static_cast<size_t>(minPts);
	// Assign cluster IDs and calculate centers of mass
	for (size_t ci = 0; ci < initial_clusters.size(); ++ci) {
		std::vector<int>& cluster_indices = initial_clusters[ci];
		// print("cluster id:", cluster_id, "contains", len(cluster_indices), "points")
		// if (cluster_indices.size() < minPts_size) {
		// 	for (std::vector<int>::size_type i = 0; i < cluster_indices.size(); ++i) {
		// 		labels[cluster_indices[i]] = -1; // Noise
		// 	}
		// 	continue;
		// }
		std::vector<point3> cluster_points;
		for (size_t i = 0; i < cluster_indices.size(); ++i) {
			cluster_points.push_back(points[cluster_indices[i]]);
			//labels[cluster_indices[i]] = cluster_id;
		}

		int x_idx = static_cast<int>((cluster_points[0].x + r) / box_size);
		int y_idx = static_cast<int>((r - cluster_points[0].y) / box_size);
		cluster_grid[y_idx][x_idx] = clusterId;

		double mass_center_x = 0.0;
		double mass_center_y = 0.0;
		size_t point_count = cluster_points.size();
		float point_count_f = static_cast<float>(point_count);

		for (size_t i = 0; i < point_count; ++i) {
			mass_center_x += cluster_points[i].x;
			mass_center_y += cluster_points[i].y;
		}
		// std::cout << "Point count: " << point_count << std::endl;
		mass_center_x /= point_count_f;
		mass_center_y /= point_count_f;
		// std::cout << "mcx: " << mass_center_x << " mcy: " << mass_center_y << std::endl;
		

		float box_centre_x = -r + (static_cast<float>(x_idx) + 0.5 )* box_size; // very important with the 0.5
		float box_centre_y = -(-r + (static_cast<float>(y_idx) + 0.5) * box_size);
		// std::cout << "bcx: " << box_centre_x << " bcy: " << box_centre_y << std::endl;

		float max_x = cluster_points[0].x;
		float min_x = cluster_points[0].x;
		float max_y = cluster_points[0].y;
		float min_y = cluster_points[0].y;

		for (size_t i = 1; i < point_count; ++i) {
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
		// std::cout << "max_x: " << max_x << " min_x: " << min_x << " max_y: " << max_y << " min_y: " << min_y << std::endl;

		// do calculations to see if there exist points in the borders
		bool point_center_vertical = false;
		bool point_center_horisontal = false;
		const float top_limit = box_centre_y + box_size / 2 * edge_limit/2;
		const float bot_limit = box_centre_y - box_size / 2 * edge_limit/2;
		const float left_limit = box_centre_x - box_size / 2 * edge_limit/2;
		const float right_limit = box_centre_x + box_size / 2 * edge_limit/2;
		for (size_t i = 0; i < point_count; ++i) {
			if (!(point_center_vertical) && (bot_limit < cluster_points[i].y < top_limit)) {
				point_center_vertical = true;
				if (point_center_horisontal && point_center_vertical) {
					break;
				}
			}
			if (!(point_center_horisontal) && (left_limit < cluster_points[i].x < right_limit)) {
				point_center_horisontal = true;
				if (point_center_horisontal && point_center_vertical) {
					break;
				}
			}
		}

		// do calculations to see if there exist points in the borders
		bool points_right = false;
		bool points_left = false;
		bool points_top = false;
		bool points_bottom = false;

		for (size_t i = 0; i < point_count; ++i) {
			if (!(point_center_horisontal)) {
				if (cluster_points[i].x > (box_centre_x + box_size / 2 * edge_limit)) {
					points_right = true;
				}
				if (cluster_points[i].x < (box_centre_x - box_size / 2 * edge_limit)) {
					points_left = true;
				}
			}

			if (!(point_center_vertical)) {
				if (cluster_points[i].y > (box_centre_y + box_size / 2 * edge_limit)) {
					points_top = true;
				}
				if (cluster_points[i].y < (box_centre_y - box_size / 2 * edge_limit)) {
					points_bottom = true;
				}
			}
		}

		// add heaviness bool
		// std::cout << "bid: (" << x_idx << ", " << y_idx << ") massx: " << mass_center_x << " lim: " << (box_centre_x + box_size * c) << " points left: " << points_left << std::endl;
		bool right_heavy = (mass_center_x > (box_centre_x + box_size * 0.5 * c)) && !points_left; // we add a center box to the heaviness
		bool left_heavy = (mass_center_x < (box_centre_x - box_size * 0.5 * c)) && !points_right;
		bool top_heavy = (mass_center_y > (box_centre_y + box_size * 0.5 * c)) && !points_bottom;
		bool bottom_heavy = (mass_center_y < (box_centre_y - box_size * 0.5 * c)) && !points_top;

		ClusterObject new_clust;
		new_clust.cluster_id = clusterId;
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
		// std::cout << "cluster id: " << new_clust.cluster_id << std::endl;
		clusterId++;
	}

	// Merge clusters based on centers of mass closer than D apart
	for (size_t index = 0; index < clusters.size(); ++index) {
		// current_box = clusters[index]
		int current_id = clusters[index].cluster_id;
		// Merge clusters based on centers of mass closer than D apart
		if (clusters[index].matched_id == -1) {
			mergeClusters(clusters, index, d, index, n, cluster_grid);
		}

		if (clusters[index].matched_id == -1) {
			for (size_t i = 0; i < clusters.size(); ++i) {
				if (clusters[i].cluster_id == current_id) {
					clusters[i].matched_id = current_id;
					// std::cout << "matched_id: " << clusters[i].matched_id << " clust id:"  << clusters[i].cluster_id << std::endl;
				}
			}
		}
	}
	

	// create new merged clusters
	// int cluster_ids = 0;
	std::vector<int> mergedMapping;
	for (size_t index = 0; index < clusters.size(); ++index) {
		int mergedMappingIndex = -1; // merged not added
		for (size_t mi=0; mi < mergedMapping.size(); ++mi) {
			int matched_id = clusters[index].matched_id;
			if (matched_id == mergedMapping[mi]) {
				mergedMappingIndex = mi;
				break;
			}
		}
		MergedClusterObject newMergedCluster;
		if (mergedMappingIndex == -1) {
			newMergedCluster.max_x =  clusters[index].max_x;
			newMergedCluster.min_x =  clusters[index].min_x;
			newMergedCluster.max_y =  clusters[index].max_y;
			newMergedCluster.min_y =  clusters[index].min_y;
		} else {
			newMergedCluster = mergedClusters[mergedMappingIndex];
		}


		// update bounding box
		if (clusters[index].max_x > newMergedCluster.max_x) {
			newMergedCluster.max_x =  clusters[index].max_x;
		}

		if (clusters[index].min_x < newMergedCluster.min_x) {
			newMergedCluster.min_x =  clusters[index].min_x;
		}

		if (clusters[index].max_y > newMergedCluster.max_y) {
			newMergedCluster.max_y =  clusters[index].max_y;
		}

		if (clusters[index].min_y < newMergedCluster.min_y) {
			newMergedCluster.min_y =  clusters[index].min_y;
		}

		// update point count
		newMergedCluster.point_count += clusters[index].point_count;
		// increment num boxes
		newMergedCluster.box_count++;

		// std::cout << "point count: " << newMergedCluster.point_count << std::endl;
		// std::cout << "box count: " << newMergedCluster.box_count << std::endl;

		newMergedCluster.mass_center_x += clusters[index].mass_center_x * clusters[index].point_count;
		newMergedCluster.mass_center_y += clusters[index].mass_center_y * clusters[index].point_count;

		if (mergedMappingIndex == -1) {
			// if not new merged cluster exist, add it
			newMergedCluster.id = clusters[index].matched_id;
			mergedClusters.push_back(newMergedCluster);
			mergedMapping.push_back(clusters[index].matched_id);
		}	else {
			mergedClusters[mergedMappingIndex] = newMergedCluster;
		}
	}
	
	// setup the dimentions for the newly created merged clusters
	for (size_t mci=0; mci < mergedClusters.size(); mci++) {
		MergedClusterObject& mc = mergedClusters[mci];
		float point_count_f = static_cast<float>(mc.point_count);
		mc.mass_center_x /= point_count_f;
		mc.mass_center_y /= point_count_f;

		mc.box_centre_x = (mc.max_x + mc.min_x) / 2.0;
		mc.box_centre_y = (mc.max_y + mc.min_y) / 2.0;
		if (mc.point_count < minPts) {
			mc.remove = true;
		}
	} // now the clusters are created and done!

	// TODO: check remove flag
}


/**
 * Creates TrackedObject instances based on the given points, labels, and unique_labels.
 *
 * @param points The vector of point3 objects representing the points.
 * @param labels The vector of integers representing the labels of the points.
 * @param unique_labels The number of unique labels.
 * @param objects The vector of TrackedObject instances to be populated.
 */
void createTrackedObjects(std::vector<MergedClusterObject>& mcos, std::vector<TrackedObject>& objects, int& maxObjectId, std::priority_queue<int, std::vector<int>, std::greater<int> >& deletedIds, float curr_time) {
	// todo handle no detections
	for (const auto mc: mcos) {
		if (mc.remove) {
			continue;
		}
		TrackedObject obj;
		obj.tlx = mc.min_x;
		obj.tly = mc.max_y;
		obj.brx = mc.max_x;
		obj.bry = mc.min_y;
		obj.numpoints = mc.point_count;
		obj.score = 0;

		if (obj.numpoints == 0) {
			continue;
		}

		obj.time_birth = curr_time;
		// obj.xpos = obj.tlx + (obj.brx - obj.tlx) / 2;
		// obj.ypos = obj.tly + (obj.bry - obj.tly) / 2;
		obj.xpos = mc.mass_center_x; // TODO: evaluate if the mass center is better
		obj.ypos = mc.mass_center_y;
		obj.area = (obj.brx - obj.tlx) * (obj.bry - obj.tly);


		// Increment objectIds and assign it to obj.id
		// TODO: add possibility to reuse deleted ids
		if (!deletedIds.empty()) {
			obj.id = deletedIds.top();
			// std::cout << "Reusing id: " << obj.id << std::endl;
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
			bool insideBBOX = (newObj.xpos < obj.brx) && (newObj.xpos > obj.tlx) && (newObj.ypos > obj.bry) && (newObj.ypos < obj.tly);
			bool areaDiffCheck = (std::abs(newObj.area - obj.area) / newObj.area) < 0.25;
			bool massCenterDistSqrdCheck = (std::pow(newObj.xpos - obj.xpos, 2)+std::pow(newObj.ypos - obj.ypos, 2)) < 9.0; // check of masscenter is within 1 meter of last seen, dist squared
			if (insideBBOX) {
				std::cout << "MATCH: oldID: " << obj.id << " , newid: " << newObj.id << std::endl;
				if (!areaDiffCheck) {
					std::cout << "areaDiffCheck NOT valid: " << std::abs(newObj.area - obj.area) / newObj.area << std::endl;	
				}

				if (!massCenterDistSqrdCheck) {
					std::cout << "massCenterDistSqrdCheck NOT valid: " << std::sqrt(std::pow(newObj.xpos - obj.xpos, 2)+std::pow(newObj.ypos - obj.ypos, 2)) << std::endl;	
				}
			}
			if ( insideBBOX && areaDiffCheck && massCenterDistSqrdCheck) { // area can shrink/grow 25%
			// float pos_dist = sqrt(pow(newObj.xpos - obj.est_xpos, 2) + pow(newObj.ypos - obj.est_ypos, 2));
			// if ( pos_dist < 5.0 ) { // if the estimated new position and the current position is less than 3m
				
				std::cout << "matched old: " << obj.id << ", new: " << newObj.id << std::endl;
				// center of new is inside old bbox!
				// TODO: improve this! and do for estimated pos instead
				// TODO: start with some points to keep tracked object for like 3 sekonds
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

		// TODO: question this
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

	// Sort 'to_delete' in descending order to prevent removing in non valid locations
	std::sort(to_delete.rbegin(), to_delete.rend());

	// Remove objects in descending order of their indices
	for (int index : to_delete) {
		deletedIds.push(allObjects[index].id);
		allObjects.erase(allObjects.begin() + index);
	}

	// std::cout << "should delete: " << to_delete.size() << std::endl;
	// // TODO: this does not work cause the array size shrinks!
	// for (size_t i=0; i < to_delete.size(); i++) {
	// 	std::cout << "deleting id: " << allObjects[to_delete[i]].id << std::endl;
	// 	deletedIds.push(allObjects[to_delete[i]].id);
	// 	allObjects.erase(allObjects.begin() + to_delete[i]);
	// 	std::cout << "t1" << std::endl;
	// }
	// std::cout << "done matching" << std::endl;



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
//     std::string wakeupMessage = "Starting MARV RADAR-Tracker v0.2 ";
//     printBox(wakeupMessage);

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
	std::ofstream obj_file("objects.csv");
	std::ofstream clust_file("clusters.csv");
	obj_file << "time,id,score,status,classification,xpos,ypos,age,est_velocity,est_heading,tlx,tly,brx,bry" << std::endl;
	clust_file << "time,id,xpos,ypos,tlx,tly,brx,bry,bcx,bcy,pc,bc" << std::endl;
	std::string line;

	std::vector<std::vector< point3 > > data;
	std::vector<std::vector< int > > labels;
	std::vector<std::vector< MergedClusterObject > > mergedClusters;
	std::vector<std::vector< float> > times;

	// Initialize new rows for each vector of vectors
	times.push_back(std::vector<float>());
	data.push_back(std::vector<point3>());
	labels.push_back(std::vector<int>());
	mergedClusters.push_back(std::vector<MergedClusterObject>());

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
				mergedClusters.push_back(std::vector<MergedClusterObject>());
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
	

	int gridsize = 27; //27
	int range = 50; //50
	float c = 0.1; //0.1
	float d = 1; //0.4
	int minPts = 40; //40
	float edge_limit = 0.5;

	// todo: configure these

	for (int r=0; r<ti+1; r++) {
		int total_points = data[r].size();
		std::cout << "Processing row: " << r << " with " << total_points << " points" << std::endl;
		// std::cout << "data length: " << data[r].size() << std::endl;
		if (data[r].size() < 1) {
			continue;
		}

		auto syst_cstart = std::chrono::system_clock::now();

		// dbscan3d(data[r], epsilon, minPts, labels[r]);
		gridScanHD(data[r], mergedClusters[r], gridsize, range, c, d, minPts, edge_limit);


		auto syst_cend = std::chrono::system_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(syst_cend - syst_cstart);
		std::cout << "CT: " << std::fixed << std::setprecision(4) << duration.count() << " ms" << std::endl;

		// Process and print label counts, comparing with ground truth
		// std::cout << "Merged clusters: " << mergedClusters[r].size() << std::endl;
		float current_time = times[r][times.size() - 1];

		// TODO: temp write clusts to csv
		//out_file << "time,id,mcx,mcy,tlx,tly,brx,bry,bcx,bcy,pc,bc,remove" << std::endl;
		if (clust_file.is_open()) {
			for (const auto& mc : mergedClusters[r]) {
				if (mc.remove) {
					continue;
				}
				clust_file << std::fixed;				
				clust_file << std::setprecision(5) << current_time << ",";	//xpos	
				clust_file << mc.id << ",";			//id						//
				clust_file << std::setprecision(5) << mc.mass_center_x << ",";	//xpos
				clust_file << std::setprecision(5) << mc.mass_center_y << ",";	//xpos
				clust_file << std::setprecision(5) << mc.min_x << ",";	//tlx = minx
				clust_file << std::setprecision(5) << mc.max_y << ",";	//tly = maxy
				clust_file << std::setprecision(5) << mc.max_x << ",";	//brx = maxx
				clust_file << std::setprecision(5) << mc.min_y << ",";	//bry = miny
				clust_file << std::setprecision(5) << mc.box_centre_x << ",";	//est_velocity
				clust_file << std::setprecision(5) << mc.box_centre_y << ",";	//est_heading
				clust_file << mc.point_count << ",";
				clust_file << mc.box_count << std::endl;
			}
		}

		// TODO: working this far!

		// -------- boxing -------
		// float current_time = times[r][times.size() - 1];
		// std::cout << "Message Time: " << current_time << std::endl;
		std::vector<TrackedObject> newObjects;
		createTrackedObjects(mergedClusters[r], newObjects, maxObjectId, deletedIds, current_time);

		// std::cout << "ids: "  << objectIds << std::endl;
		// std::cout << "Objects: "  << unique_labels << std::endl;

		// for (int o=0; o<newObjects.size(); o++) {
		// 	TrackedObject obj = newObjects[o];
		// 	std::cout << "ID: " << obj.id << " pts: " << obj.numpoints << " x: " << obj.xpos << " y: " << obj.ypos;
		// 	std::cout << " tlx: " << obj.tlx << std::endl;
		// }

		// --------- Matching ----------
		matchObjects(allObjects, newObjects, deletedIds);
		std::cout << "done matching" << std::endl;

		std::chrono::time_point<std::chrono::system_clock> syst_mend = std::chrono::system_clock::now();
		std::chrono::duration<double> duration_2 = std::chrono::duration_cast<std::chrono::duration<double>>(syst_mend - syst_cend);

		std::cout << "MT: " << std::fixed << std::setprecision(4) << duration_2.count() << std::endl;


		// auto syst_mend = std::chrono::system_clock::now();
		// auto duration_2 = std::chrono::duration_cast<std::chrono::duration<double>>(syst_mend - syst_cend);

		// std::cout << ", MT: " << std::fixed << std::setprecision(4) << duration_2.count() << ", T: " << std::setprecision(4) << current_time << std::endl;

		// std::cout << "All objects: " << allObjects.size() << std::endl;
		// for (const auto& ao : allObjects) {
		// 	std::cout << "ID: " << ao.id << " score: " << ao.score << " Velocity: " << ao.est_vel << " Heading: " << ao.est_heading << std::endl;
		// }
		// std::cout << "-----------------" << std::endl;

		// // TODO: reuse ids

		// "time,id,score,status,classification,xpos,ypos,age,est_velocity,est_heading,tlx,tly,brx,bry"
		if (obj_file.is_open()) {
			for (const auto& ao : allObjects) {
				obj_file << current_time << ","; 	//time
				obj_file << ao.id << ",";			//id
				obj_file << ao.score << ",";		//score
				obj_file << ao.status << ",";		//status
				obj_file << ao.classification << ",";	//classification
				obj_file << std::fixed;								//
				obj_file << std::setprecision(5) << ao.xpos << ",";	//xpos
				obj_file << std::setprecision(5) << ao.ypos << ",";	//ypos
				obj_file << std::setprecision(5) << ao.age << ",";	//age
				obj_file << std::setprecision(5) << ao.est_vel << ",";	//est_velocity
				obj_file << std::setprecision(5) << ao.est_heading << ",";	//est_heading
				obj_file << std::setprecision(5) << ao.tlx << ",";			//tlx
				obj_file << std::setprecision(5) << ao.tly << ",";			//tly
				obj_file << std::setprecision(5) << ao.brx << ",";			//brx
				obj_file << std::setprecision(5) << ao.bry << std::endl;	//bry
				// out_file << current_time << "," << ao.id << "," << ao.score << "," << ao.est_vel << "," << ao.est_heading << "\n";
			}
		}

		

		std::chrono::time_point<std::chrono::system_clock> syst_tend = std::chrono::system_clock::now();
		std::chrono::duration<double> duration_end = std::chrono::duration_cast<std::chrono::duration<double>>(syst_tend - syst_cstart);

		std::cout << "Total: " << std::fixed << std::setprecision(4) << duration_end.count()*1000 << " ms" << std::endl;

	}
	obj_file.close();
	clust_file.close();
	return 0;
}
