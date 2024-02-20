
#include <chrono>
// #include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

#include "rclcpp/qos.hpp"
#include "radar_tracker/exception.hpp"
#include "radar_tracker/interfaces/lifecycle_interface.hpp"
#include "radar_tracker/radar_tracker.hpp"

namespace radar_tracker
{
	using namespace std::chrono_literals;
	using std::placeholders::_1;
	using std::placeholders::_2;
	using std::placeholders::_3;
	using std::placeholders::_4;

	RadarTracker::RadarTracker(
		const rclcpp::NodeOptions & options)
	: LifecycleInterface("RadarTracker", options)
	{
		// Declare parameters for configuring the _driver_
		this->declare_parameter("use_system_default_qos", true);

		// Declare parameters used across ALL _sensor_ implementations
		this->declare_parameter("min_range", 1);
		this->declare_parameter("max_range", 450);
		this->declare_parameter("min_intensity", 0.8);
		this->declare_parameter("min_points", 16);
		this->declare_parameter("epsilon", 6.0);
		this->declare_parameter("timestamp_mode", "TIME_FROM_ROS_RECEPTION");
		// TODO: warning sbg time is only in seconds!
	}

	RadarTracker::~RadarTracker() = default;

	void RadarTracker::onConfigure()
	{
		// Get parameters for configuring the _driver_
		_use_system_default_qos = get_parameter("use_system_default_qos").as_bool();

		// Get parameters used across ALL _sensor_ implementations. Parameters unique
		// a specific Sensor implementation are "getted" in the configure() function
		// for that sensor.
		//radar_tracker::Configuration tracker_config;

		tracker_config.min_intensity = this->get_parameter("min_intensity").as_double();
		// TODO: evaluate if do 3d clustering
		tracker_config.min_range = this->get_parameter("min_range").as_int();
		tracker_config.max_range = this->get_parameter("max_range").as_int();

		tracker_config.min_points = this->get_parameter("min_points").as_int();
		tracker_config.epsilon = this->get_parameter("epsilon").as_double();
		tracker_config.timestamp_mode = this->get_parameter("timestamp_mode").as_string();

		// Print the values nicely formatted
		std::cout << "Parameter Values:" << std::endl;
		std::cout << "-----------------" << std::endl;
		std::cout << "Min Intensity: " << tracker_config.min_intensity << std::endl;
		std::cout << "Min Range: " << tracker_config.min_range << std::endl;
		std::cout << "Max Range: " << tracker_config.max_range << std::endl;
		std::cout << "Min Points: " << tracker_config.min_points << std::endl;
		std::cout << "Epsilon: " << tracker_config.epsilon << std::endl;
		std::cout << "Timestamp Mode: " << tracker_config.timestamp_mode << std::endl;


		if (tracker_config.timestamp_mode == "TIME_FROM_ROS_RECEPTION") {
			RCLCPP_WARN(
				this->get_logger(),
				"Using TIME_FROM_ROS_RECEPTION to stamp data with ROS time on "
				"reception. This has unmodelled latency!");
			tracker_config.timestamp_mode = "TIME_FROM_INTERNAL_OSC";
			_use_ros_time = true;
		} else {
			_use_ros_time = false;
		}

		//_tracker_detection_points = this->create_publisher<tracker_msgs::msg::tracker_detection>(
		//  "/radar_tracker/TrackerDetection", rclcpp::SystemDefaultsQoS());
		// initially do not publish detections
		// Part 1. 
		// TODO: maybe change to radar driver msgs
		// _radar_driver_subscriber = this->create_subscription<tracker_msgs::msg::RadarScanCartesian>(
		// "/radar_driver/RadarScanCartesian", 10, std::bind(&RadarTracker::processDetections, this, std::placeholders::_1));
		_radar_driver_subscriber = this->create_subscription<radar_msgs::msg::RadarScanCartesian>(
		"/radar_driver/RadarScanCartesian", 10, std::bind(&RadarTracker::processDetections, this, std::placeholders::_1));



		_tracker_cluster_labels = this->create_publisher<std_msgs::msg::Int32>(
		"/radar_tracker/ClusterLabels", rclcpp::SystemDefaultsQoS());

		// Procedure:
		// 1. get detections from radar driver
		// 2. cluster detections
		// 3. reverse calculate distance traveled with time diff and gps
		// 4. iterate to do object fusing
		// 5. publish current objects

		// start detection processing thread, run once every second and take full rotation and filter for range and intensity
		// then pubish store buffer of these detections x,y .... for tracker to use.



		/// ---------------------------
		// _detections_buf = std::make_unique<RingBuffer>(10, 500000); // x,y,intensity,doppler
		// _detections_buf_read_lock = false;

		// 'x':np.empty(0, dtype=np.float32), 
		// 'y':np.empty(0, dtype=np.float32), 	
		// 'intensity':np.empty(0, dtype=np.uint8), 0-15
		// 'doppler':np.empty(0, dtype=np.uint8) 0-15
		//_detection_thread = std::thread(std::bind(&RadarTracker::processDetections, this));



		// _process_detections = true;

		// start tracker thread
		// take detections and compare with previous to create tracking.
		// _tracked_objects = std::make_unique<RingBuffer>(50, 500);
		// "num_points":np.empty(num_objects,dtype=np.float32),
		// "xpos":np.empty(num_objects,dtype=np.float32),
		// "ypos":np.empty(num_objects,dtype=np.float32),
		
		// "tlx":np.empty(num_objects,dtype=np.float32),
		// "tly":np.empty(num_objects,dtype=np.float32),
		// "brx":np.empty(num_objects,dtype=np.float32),
		// "bry":np.empty(num_objects,dtype=np.float32),

		// "doppler":np.empty(num_objects,dtype=np.float32),
		// status: uint8
		// id: int 32
		// score int32
		// max_score int32
		// head: float32
		// vel: float32
		// est_xpos: float32
		// est_ypos: float32
		// est_tlx: float32
		// est_tly: float32
		// est_brx: float32
		// est_bry: float32
		// last_update: int32
		// classification: int32 
		// _run_tracking = true; // IE start tracking immidiately
		// _tracking_thread = std::thread(std::bind(&RadarTracker::runTrackIt, this));

		RCLCPP_INFO(
			this->get_logger(),
			"This driver is compatible with NavicoRadar sensors");

		_reset_srv = this->create_service<std_srvs::srv::Empty>(
			"~/reset", std::bind(&RadarTracker::resetService, this, _1, _2, _3));

	}
	

	void RadarTracker::onActivate()
	{
		
		_detections_buf = std::make_unique<RingBuffer>(10, 200000); // x,y,intensity,doppler
		// TODO; change buf to 500_000
		_detections_buf_read_lock = false;
		_process_detections = true;
		_tracked_objects = std::make_unique<RingBuffer>(50, 500);
		_tracker_cluster_labels->on_activate();
		_run_tracking = true;
		_tracking_thread = std::thread(std::bind(&RadarTracker::runTrackIt, this));
	}

	void RadarTracker::onError()
	{
	}

	void RadarTracker::onDeactivate()
	{	
		_run_tracking = false;

		if (_tracking_thread.joinable()) {
		_tracking_thread.join();
	}
		// if ( _tracking_thread.joinable() ) 
		// {
			// _tracking_thread.join();
		// }
	}

	void RadarTracker::onCleanup()
	{

	}

	void RadarTracker::onShutdown()
	{
		this->onDeactivate();
	}

	// const std::shared_ptr<tracker_msgs::msg::RadarScanCartesian>
	// void RadarTracker::processDetections(const tracker_msgs::msg::RadarScanCartesian::SharedPtr msg)
	void RadarTracker::processDetections(const radar_msgs::msg::RadarScanCartesian::SharedPtr msg)
	{	
		// TODO: we do not get to here
		Detection dect;
		// if (_process_detections) {
			try {
				std::cout << "got radar scan. buffer size: " <<  _detections_buf->size() << std::endl;
				if (!(_detections_buf->full())) 
				{
					// int num_of_detections = sizeof(msg->x_indices);
					int num_of_detections = msg->detections;
					std::cout << "num of detections: " << num_of_detections << std::endl;
					// _detections_buf_read_lock = true; // for threading
					// int msg_detections = msg->detections;
					std::cout << "num of detections from msg: " << num_of_detections << ". first coord: " << msg->x_indices[0] << std::endl;
					
					if (num_of_detections > 8 ) {
						for (int i=0; i<8;i++) 
						{ 
							std::cout << "X: " << msg->x_indices[i] << " Y: " << msg->y_indices[i] 
							<< " I: " << static_cast<int>(msg->intensities[i]) << " R: " << msg->ranges[i] << std::endl;
						}
					}




					for (int i=0; i<num_of_detections;i++) {
						int intens = static_cast<int>(msg->intensities[i]);
						bool detection_filter = true;
						detection_filter &= (msg->ranges[i] >= tracker_config.min_range);
						detection_filter &= (msg->ranges[i] <= tracker_config.max_range);
						detection_filter &= (static_cast<float>(intens)/255 >= tracker_config.min_intensity);

						if (detection_filter){ // TODO: check correctness, add range filter
							dect.x = msg->x_indices[i];
							dect.y = msg->y_indices[i];
							dect.intensity = msg->intensities[i];
							// TODO: msut check if doppler is active
							// dect.doppler = msg->doppler_returns[i];
							// dect = {msg->x_indices[i], msg->y_indices[i], msg->intensities[i], msg->doppler_returns[i]};
							uint8_t* tail_ptr = _detections_buf->tail();
							// uint8_t * buf



							if (msg == nullptr || _detections_buf == nullptr || tail_ptr == nullptr) {
								// Handle error or return
								RCLCPP_WARN(
								this->get_logger(),
								"nullptr exeption in processDetections.");
							}
							memcpy(tail_ptr, &dect, 10);

							_detections_buf->push();
						}
					}
					_detections_buf_read_lock = false;

					std::cout << "done with push" << std::endl;
					

					// RCLCPP_INFO( // TODO: Debug
					// 	this->get_logger(),
					// 	"I got scan iteration from detection process");
				} else {

					RCLCPP_WARN(
						this->get_logger(),
						"The detections buffer is full, dropping scan iteration.");
				}
			} catch (const RadarTrackerException & e) {
				RCLCPP_WARN(
					this->get_logger(),
					"Failed to process packet with exception %s.", e.what());
			}
		// }
	}

	void RadarTracker::runTrackIt()
	{
		uint64_t last_iteration_time = 0; //NOTE: used for dt
		uint64_t time_diff_ns; //NOTE: used for dt
		float time_diff = 1.;
		uint64_t sys_time;
		
		RCLCPP_INFO(this->get_logger(), "Running tracking thread.");
		while (_run_tracking) {
			try {

				// if (!(_detections_buf_read_lock) && !(_detections_buf->empty())) {	
				if (!(_detections_buf->empty())) {	
					// std::cout << "Running tracking io" << std::endl;
					// check sys time exists
					sys_time = this->now().nanoseconds();

					if (last_iteration_time == 0) {
						time_diff = 1.;
					} else {
						time_diff_ns = sys_time - last_iteration_time; // ns!
						time_diff = static_cast<float>(time_diff_ns) * pow(10, -9);
					}

		
					RCLCPP_INFO(this->get_logger(), "Time diff: %f", time_diff);

					last_iteration_time = sys_time;		
						
					
					// Part 2.
					RadarTracker::clusterFrame();

					// Part 3.

				}
				
			} catch (const RadarTrackerException & e) {
				RCLCPP_WARN(
					this->get_logger(),
					"Failed to track with exception %s.", e.what());
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 0.2 secs
		}
		RCLCPP_INFO(
		this->get_logger(),
		"Exiting tracking thread.");
		
	}

	void RadarTracker::clusterFrame()
	{
		
		std::cout << "Buf size " << _detections_buf->size() << std::endl; // oke!
		if (_detections_buf->empty()) return;

		std::size_t np = _detections_buf->size();
		std::vector<point3> points;
		points.reserve(np);
		point3 tmp_point;
		while (!(_detections_buf->empty())) { // empty the whole buffer!
			// Cast the head of the buffer to a Detection pointer
			Detection* dect2 = reinterpret_cast<Detection*>(_detections_buf->head());
	
			tmp_point.x = static_cast<double>(dect2->x);
			tmp_point.y = static_cast<double>(dect2->y);
			tmp_point.z = static_cast<double>(dect2->intensity);
			// Now you can access the Detection object's fields
			// For example: dect->x, dect->y, dect->z
			points.push_back(tmp_point);
			// Remember to pop the head after you're done processing this Detection
			_detections_buf->pop();
		}
		
		std::cout << "Running clustering with:" << np << " points. First point: " << points[0].x << std::endl;
		// TODO: continue here

		std::vector<int> labels = dbscan3d(points, tracker_config.epsilon, tracker_config.min_points);
		std::vector<int> uniqueValues;

		// Sort the original vector
		std::sort(labels.begin(), labels.end());

		// Use std::unique_copy to copy unique elements to the new vector
		std::unique_copy(labels.begin(), labels.end(), std::back_inserter(uniqueValues));
		
		int num_classes = uniqueValues.size()-1; // 0 is noise.
		if (num_classes <= 0) {
			std::cout << "No classes found" << std::endl;
			return;
		}
		std::cout << "num classes from clustering: " << num_classes << std::endl;

		uniqueValuesMsg.data = num_classes;
		_tracker_cluster_labels->publish(uniqueValuesMsg);

		// ------------- start building object ----------
		// order is like:
		// x_0
		// y_0
		// label_0 ...
		std::vector<TrackedObject> objects(num_classes);
		int lbl = 0;
		float x = 0.0;
		float y = 0.0;
		for (int i=0; i<labels.size();i++) {
			if (labels.data()[i]!=0) { // skip noice label 0
				// check how to calc mid point
				lbl = labels.data()[i] - 1; // 1 becomes index zero
				x = points[i].x;
				y = points[i].y;
				z = points[i].z;

				// Update min and max x and y values
				min_x = std::min(objects[lbl].tlx, x);
				max_x = std::max(objects[lbl].brx, x);
				min_y = std::min(objects[lbl].tly, y);
				max_y = std::max(objects[lbl].bry, y);

				// Update the midpoint of objects[lbl]
				objects[lbl].xpos = (min_x + max_x) / 2.0;
				objects[lbl].ypos = (min_y + max_y) / 2.0;
				objects[lbl].numpoints ++;
				objects[lbl].tlx = min_x;
				objects[lbl].tly = min_y;
				objects[lbl].brx = max_x;
				objects[lbl].bry = max_y;
				// TODO: dont forget doppler vals/direction
			
			}
		}
	}

	// void RadarTracker::estimate_object_position( INS_data, tracked_objects, elapsed_time)
	// {
	// 	// Update estimated position of objects by dead reckoning
	// 	// If no recorded velocity/heading, estimated position is set to detected position
	// 	// Also accounts for movement of vessel

	// 	// Set estimated values of objects with recorded velocities by dead reckoning

	// 	// Account for distance travelled in true and estimated position
	// 	float x_diff, y_diff, _;
	// 	std::tie(x_diff, y_diff, _) = INS_data;

	// 	for (int i = 0; i < tracked_objects.size(); i++) {
	// 		float tcos = 0.0;
	// 		float tsin = 0.0;
	// 		if (tracked_objects["vel"][i] != -1) {
	// 			float tcos = tracked_objects["vel"][i] * cos(tracked_objects["heading"][i]) * elapsed_time;
	// 			float tsin = tracked_objects["vel"][i] * sin(tracked_objects["heading"][i]) * elapsed_time;
	// 		}
	// 			tracked_objects[i].est_xpos = tracked_objects[i].xpos + tcos;
	// 			tracked_objects[i].est_ypos = tracked_objects[i].ypos + tsin;
	// 			tracked_objects[i].est_tlx = tracked_objects[i].tlx + tcos;
	// 			tracked_objects[i].est_tly = tracked_objects[i].tly + tsin;
	// 			tracked_objects[i].est_brx = tracked_objects[i].brx + tcos;
	// 			tracked_objects[i].est_bry = tracked_objects[i].bry + tsin;
				
	// 			// update pos
	// 			tracked_objects[i].xpos -= x_diff;
	// 			tracked_objects[i].est_xpos -= x_diff;
	// 			tracked_objects[i].ypos -= y_diff;
	// 			tracked_objects[i].est_ypos -= y_diff;		
	// 	}
	// }

	

	void RadarTracker::resetService(
		const std::shared_ptr<rmw_request_id_t>/*request_header*/,
		const std::shared_ptr<std_srvs::srv::Empty::Request> request,
		std::shared_ptr<std_srvs::srv::Empty::Response> response)
	{
		if (!this->isActive()) {
			return;
		}

		
		radar_tracker::Configuration tracker_config;

		tracker_config.min_intensity = this->get_parameter("min_intensity").as_double();
		// TODO: evaluate if do 3d clustering
		tracker_config.min_range = this->get_parameter("min_range").as_int();
		tracker_config.max_range = this->get_parameter("max_range").as_int();


		tracker_config.min_points = this->get_parameter("min_points").as_int();
		tracker_config.epsilon = this->get_parameter("epsilon").as_double();
		tracker_config.timestamp_mode = this->get_parameter("timestamp_mode").as_string();

	}

}  // namespace radar_tracker
