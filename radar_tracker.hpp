

#ifndef RADAR_ROS2__RADAR_TRACKER_HPP_
#define RADAR_ROS2__RADAR_TRACKER_HPP_

#include <memory>
#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <vector>

#include "radar_tracker/dbscan3d.hpp"

// #include "radar_tracker/conversions.hpp"

#include "radar_tracker/interfaces/lifecycle_interface.hpp"

// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "tracker_msgs/msg/radar_scan_cartesian.hpp"
#include "radar_msgs/msg/radar_scan_cartesian.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/int32.hpp" // very important include


// #include "tf2_ros/static_transform_broadcaster.h"

#include "radar_tracker/interfaces/configuration.hpp"
#include "radar_tracker/radar_info.hpp"

// TODO: remove?
#include "radar_tracker/ringbuffer.hpp"

namespace radar_tracker
{

// class SensorInterface;

/**
 * @class radar_tracker::RadarTracker
 * @brief A lifecycle interface implementation of a Radar
 * driver in ROS2.
 */
class RadarTracker : public lifecycle_interface::LifecycleInterface
{
public:
  // using DataProcessorMap = std::multimap<radar::sensor::client_state,
  //     std::unique_ptr<radar_tracker::DataProcessorInterface>>;
  // using DataProcessorMapIt = DataProcessorMap::iterator;

  /**
   * @brief A constructor for radar_tracker::RadarTracker
   * @param options Node options for lifecycle node interfaces
   */
  RadarTracker(
    // std::unique_ptr<SensorInterface> sensor,
    const rclcpp::NodeOptions & options);

  /**
   * @brief A destructor for radar_tracker::RadarTracker
   */
  ~RadarTracker();

  /**
   * @brief lifecycle node's implementation of configure step
   * which will configure ROS interfaces and allocate resources
   */
  void onConfigure() override;

  /**
   * @brief lifecycle node's implementation of activate step
   * which will activate ROS interfaces and start processing information
   */
  void onActivate() override;

  /**
   * @brief lifecycle node's implementation of deactivate step
   * which will deactivate ROS interfaces and stop processing information
   */
  void onDeactivate() override;

  /**
   * @brief lifecycle node's implementation of error step
   * which will handle errors in the lifecycle node system
   */
  void onError() override;

  /**
   * @brief lifecycle node's implementation of shutdown step
   * which will shut down the lifecycle node
   */
  void onShutdown() override;

  /**
   * @brief lifecycle node's implementation of cleanup step
   * which will deallocate resources
   */
  void onCleanup() override;

private:

  /**
  * @brief service callback to reset the radar
  * @param request_header Header of rmw request
  * @param request Shared ptr of the Empty request
  * @param response Shared ptr of the Empty response
  */
  void resetService(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);


  /**
  * @brief Thread function to process buffered packets that have been received from the sensor
  */
  void processDetections(const radar_msgs::msg::RadarScanCartesian::SharedPtr msg);
	//void processDetections(const tracker_msgs::msg::RadarScanCartesian::SharedPtr msg);

  /**
  * @brief Thread function to keep sensor alive
  */
  void runTrackIt();

  /**
  * @brief Thread function to keep sensor alive
  */
  void clusterFrame();

// // noise will be labelled as 0
// std::vector<size_t> label(const std::vector<std::vector<size_t>>& clusters, size_t n)
// {
//     std::vector<size_t> flat_clusters(n);

//     for(size_t i = 0; i < clusters.size(); i++)
//     {
//         for(auto p: clusters[i])
//         {
//             flat_clusters[p] = i + 1;
//         }
//     }

//     return flat_clusters;
// }

  // reports -------------
  #define NAVICO_SPOKE_LEN 1024
#pragma pack(push, 1)

struct Detection {  // 01 C4 with length 18
  float x;               // 0   0x01
  float y;            // 1   0xC4
  uint8_t intensity;       // 2
  uint8_t doppler;             // 3
};

struct TrackedObject {  // 01 C4 with length 18
  uint32_t numpoints;
  
  float xpos;
  float ypos; //TODO: need to be int?
  
  float tlx = std::numeric_limits<float>::max();
  float tly = std::numeric_limits<float>::lowest();
  float brx = std::numeric_limits<float>::max();
  float bry = std::numeric_limits<float>::lowest();

  uint8_t doppler_dir;
  uint8_t status; //0: unconfirmed, 1: confirmed, 2: deleted
  uint32_t id;
  float score;
  float max_score;

  float heading;
  float vel;

  float est_xpos;
  float est_ypos;

  float est_tlx;
  float est_tly;
  float est_brx;
  float est_bry;

  uint32_t last_update;
  uint8_t classification; // 0: undefined, 1: stationary, 2: moving
  uint32_t age;

};
#pragma pack(pop)


  // ---------------------------

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_srv;

  std::string _laser_sensor_frame, _laser_data_frame;
  // std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _tf_b;

  bool _use_system_default_qos;
  bool _use_ros_time;
  radar_tracker::RadarInfo ri;
  radar_tracker::RadarInfoRaw rir;

  radar_tracker::Configuration tracker_config;

  std::uint32_t _proc_mask;

  // Ringbuffers for raw received radar packets
  std::unique_ptr<RingBuffer> _detections_buf;
  std::unique_ptr<RingBuffer> _tracked_objects;

  // Threads and synchronization primitives for receiving and processing data
  std::thread _tracking_thread;
//   std::condition_variable _process_cond;
//   std::mutex _ringbuffer_mutex;
  bool _run_tracking;
  bool _process_detections;
  bool _detections_buf_read_lock;
  
//   bool _heartbeat_active;
//   bool _parse_reports;
//   bool _parse_frames;
//   bool _publish_scan;
  std_msgs::msg::Int32 uniqueValuesMsg;
//   tracker_msgs::msg::RadarScanCartesian _radar_scan_cartesian;
	radar_msgs::msg::RadarScanCartesian _radar_scan_cartesian;
  
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr _tracker_cluster_labels;

//   rclcpp::Subscription<tracker_msgs::msg::RadarScanCartesian>::SharedPtr _radar_driver_subscriber;
  rclcpp::Subscription<radar_msgs::msg::RadarScanCartesian>::SharedPtr _radar_driver_subscriber;


};

}  // namespace radar_tracker

#endif  // RADAR_ROS2__RADAR_TRACKER_HPP_
